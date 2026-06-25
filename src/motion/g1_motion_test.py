#!/usr/bin/env python3
"""
g1_motion_test.py — G1 動作テスト用インタラクティブ CLI

run_sim.sh + run_deploy.sh 起動後に実行。
対話不要で動作・歩行を直接テストできる。

動作データは data/motions/<name>/sample_N.npz から読み込みます。
全エントリはフラットな番号付きリストとして管理します。

操作:
  [raw モード (1キー即時)]
    w/s/a/d  : 歩行
    Space    : 停止
    1〜9    : エントリ番号で即時再生
    n / p    : 次 / 前のエントリを再生
    f        : 次のキーを「強制再生」にする (例: f3)
    l        : 動作一覧
    h / ?    : ヘルプ
    Enter    : コマンド入力モードへ
    Ctrl+C   : 終了

  [コマンド入力モード]
    w [n]  s [n]  a [n]  d [n]  : 歩行
    stop / x            : 停止
    <番号>              : エントリ番号で再生
    <名前>              : s1 を再生 (部分一致可)
    <名前> <s>          : 指定サンプルを再生   例: nod 2
    n / next  p / prev  : 次 / 前のエントリ
    f <cmd>             : 強制中断して実行
    list / l            : 動作一覧
    status              : 状態確認
    q                   : 終了
    ESC                 : raw モードへ戻る
"""

import argparse
import json
import os
import select as _select
import signal
import struct
import sys
import termios
import threading
import time
import tty
from pathlib import Path
from typing import Optional

import numpy as np
import zmq

# ── 設定 ──────────────────────────────────────────────────────
ZMQ_PORT    = 5556
HEADER_SIZE = 1280
SONIC_FPS   = 50
CHUNK_SIZE  = 5
WALK_STEP_DURATION   = 0.35
TURN_REPEAT_COUNT    = 5
TURN_REPEAT_INTERVAL = 0.05

REPO_ROOT      = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
G1_MOTION_DIR  = os.path.join(REPO_ROOT, "data", "motions")

# 動作説明 (generate_motions.py の MOTIONS から)
_MOTION_DESCS: dict[str, str] = {
    "idle": "直立待機", "at_ease": "休憩姿勢",
    "nod": "うなずき", "deep_nod": "大きくうなずき",
    "thumbs_up": "右親指を立てる", "double_thumbs_up": "両親指を立てる",
    "hand_on_chest": "胸に手を当てる", "point_to_self": "自分を指す",
    "wave_off": "手を振って否定", "cross_arms_x": "両腕でX（ダメ）",
    "lean_back_surprised": "後ろに反らして驚き",
    "bow_slight": "軽いお辞儀(15°)", "bow_45": "お辞儀(45°)", "bow_deep": "深いお辞儀",
    "wave": "右手を振る", "salute": "敬礼", "namaste": "合掌",
    "handshake_offer": "握手の手を差し出す", "welcome_arms": "両腕で歓迎",
    "point_forward": "前を指差す", "point_left": "左を指差す",
    "point_right": "右を指差す", "point_up": "上を指差す",
    "point_down": "下を指差す", "point_back_over_shoulder": "肩越しに後方を指す",
    "arms_open": "両腕を広げて案内", "this_way_right": "右方向へ案内",
    "this_way_left": "左方向へ案内", "present_with_both_hands": "両手で提示",
    "beckon": "手招き", "halt": "手のひらで停止",
    "shrug": "肩をすくめる", "clap": "拍手", "banzai": "万歳",
    "fist_pump": "ガッツポーズ", "lean_forward_interest": "前傾して興味",
    "arms_akimbo": "腰に手を当てる",
    "bow_apology": "謝罪のお辞儀", "hands_together_apology": "両手を合わせ謝罪",
}


# ── Kimodo 動作ライブラリ読み込み ──────────────────────────────

def _compute_jv(jp: np.ndarray) -> np.ndarray:
    jv = np.zeros_like(jp)
    if len(jp) > 1:
        jv[:-1] = (jp[1:] - jp[:-1]) * SONIC_FPS
        jv[-1] = jv[-2]
    return jv


def load_motions(motion_dir: str = G1_MOTION_DIR) -> list[dict]:
    """data/motions/<name>/sample_N.npz を全スキャンしてフラットリストを返す。"""
    root = Path(motion_dir)
    if not root.exists():
        print(f"[Motion] ディレクトリが見つかりません: {motion_dir}")
        return []
    entries: list[dict] = []
    for name_dir in sorted(root.iterdir()):
        if not name_dir.is_dir():
            continue
        name = name_dir.name
        for npz in sorted(name_dir.glob("sample_*.npz")):
            try:
                sample_num = int(npz.stem.split("_")[1])
                d  = np.load(npz)
                jp = d["jp"].astype(np.float32)
                jv = (d["jv"].astype(np.float32) if "jv" in d
                      else _compute_jv(jp))
                bq = (d["bq"].astype(np.float32) if "bq" in d
                      else np.tile([1., 0., 0., 0.], (len(jp), 1)).astype(np.float32))
                entries.append({
                    "key":     f"{name}/s{sample_num}",
                    "name":    name,
                    "sample":  sample_num,
                    "desc":    _MOTION_DESCS.get(name, name),
                    "jp": jp, "jv": jv, "bq": bq,
                    "dur_sec": len(jp) / SONIC_FPS,
                })
            except Exception as e:
                print(f"[Motion] ✗ {name}/{npz.name}: {e}")
    print(f"[Motion] {len(entries)} エントリ ロード完了\n")
    return entries


# ── ZMQ 送信 ──────────────────────────────────────────────────

def send_pose(sock, joint_pos, joint_vel, body_quat, frame_index):
    N = len(joint_pos)
    header = {
        "v": 1, "endian": "le", "count": N,
        "fields": [
            {"name": "joint_pos",   "dtype": "f32", "shape": [N, 29]},
            {"name": "joint_vel",   "dtype": "f32", "shape": [N, 29]},
            {"name": "body_quat_w", "dtype": "f32", "shape": [N, 4]},
            {"name": "frame_index", "dtype": "i64", "shape": [N]},
            {"name": "catch_up",    "dtype": "u8",  "shape": [1]},
        ]
    }
    hj = json.dumps(header).encode()
    hb = hj + b"\x00" * (HEADER_SIZE - len(hj))
    fi = np.arange(frame_index, frame_index + N, dtype=np.int64)
    data = (joint_pos.tobytes() + joint_vel.tobytes() +
            body_quat.tobytes() + fi.tobytes() + struct.pack("B", 0))
    sock.send(b"pose" + hb + data)


# ── MotionPlayer ──────────────────────────────────────────────

class MotionPlayer:
    def __init__(self, sock):
        self._sock   = sock
        self._thread: Optional[threading.Thread] = None
        self._stop   = threading.Event()
        self._fi     = 0

    def is_playing(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def play_once(self, motion: dict, force: bool = False):
        if not force and self.is_playing():
            return
        self._stop.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=2.0)
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, args=(motion,), daemon=True)
        self._thread.start()

    def stop(self):
        self._stop.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=2.0)

    def _run(self, motion: dict):
        jp, jv, bq = motion["jp"], motion["jv"], motion["bq"]
        T = len(jp)
        for i in range(0, T, CHUNK_SIZE):
            if self._stop.is_set():
                return
            n  = min(CHUNK_SIZE, T - i)
            t0 = time.perf_counter()
            send_pose(self._sock, jp[i:i+n], jv[i:i+n], bq[i:i+n], self._fi)
            self._fi += n
            wait = n / SONIC_FPS - (time.perf_counter() - t0)
            if wait > 0:
                self._stop.wait(timeout=wait)


# ── WalkerController ──────────────────────────────────────────

class WalkerController:
    def __init__(self, sock):
        self._sock         = sock
        self._facing_angle = 0.0
        self._TURN_STEP    = np.radians(10)
        self._planner_mode = False
        self._lock         = threading.Lock()
        self._action_stop  = threading.Event()
        self._action_thread: Optional[threading.Thread] = None

    def _send_msg(self, topic, fields, data):
        header = {"v": 1, "endian": "le", "count": 1, "fields": fields}
        hj = json.dumps(header).encode()
        hb = hj + b"\x00" * (HEADER_SIZE - len(hj))
        self._sock.send(topic + hb + data)

    def send_command(self, start=True, stop=False, planner=True):
        fields = [
            {"name": "start",   "dtype": "u8", "shape": [1]},
            {"name": "stop",    "dtype": "u8", "shape": [1]},
            {"name": "planner", "dtype": "u8", "shape": [1]},
        ]
        self._send_msg(b"command", fields, struct.pack("BBB", int(start), int(stop), int(planner)))

    def send_planner(self, mode, movement, facing, speed=-1.0):
        fields = [
            {"name": "mode",     "dtype": "i32", "shape": [1]},
            {"name": "movement", "dtype": "f32", "shape": [3]},
            {"name": "facing",   "dtype": "f32", "shape": [3]},
            {"name": "speed",    "dtype": "f32", "shape": [1]},
            {"name": "height",   "dtype": "f32", "shape": [1]},
        ]
        data  = struct.pack("<i", mode)
        data += struct.pack("<fff", *movement)
        data += struct.pack("<fff", *facing)
        data += struct.pack("<ff", speed, -1.0)
        self._send_msg(b"planner", fields, data)

    def _fv(self):
        a = self._facing_angle
        return [np.cos(a), np.sin(a), 0.0]

    def _wait_or_stop(self, seconds: float) -> bool:
        return self._action_stop.wait(timeout=seconds)

    def _cancel_action(self, wait: bool = True):
        self._action_stop.set()
        if wait and self._action_thread and self._action_thread.is_alive():
            self._action_thread.join(timeout=1.0)
        self._action_thread = None
        self._action_stop.clear()

    def run_action(self, action):
        self._cancel_action(wait=True)
        self._action_thread = threading.Thread(target=action, daemon=True)
        self._action_thread.start()

    def start_planner(self):
        with self._lock:
            if not self._planner_mode:
                self.send_command(start=True, stop=False, planner=True)
                if self._wait_or_stop(1.0):
                    return
                self.send_planner(0, [0, 0, 0], self._fv())
                self._planner_mode = True
                print("[Walker] planner モード開始")

    def _ensure_planner(self):
        if not self._planner_mode:
            self.send_command(start=True, stop=False, planner=True)
            if self._wait_or_stop(0.3):
                return False
            self._planner_mode = True
        return True

    def _walk_linear(self, sign: float, steps: int = 1):
        steps = max(1, int(steps))
        with self._lock:
            if not self._ensure_planner():
                return False
            for i in range(steps):
                a = self._facing_angle
                mv = [sign * np.cos(a), sign * np.sin(a), 0.0]
                self.send_planner(2, mv, self._fv())
                if self._wait_or_stop(WALK_STEP_DURATION):
                    self.send_planner(0, [0, 0, 0], self._fv())
                    return False
                self.send_planner(0, [0, 0, 0], self._fv())
                if i < steps - 1 and self._wait_or_stop(0.08):
                    return False
        return True

    def walk_forward(self, steps: int = 1):
        if self._walk_linear(1.0, steps):
            print(f"[Walker] 前進 x{max(1, int(steps))}")

    def walk_backward(self, steps: int = 1):
        if self._walk_linear(-1.0, steps):
            print(f"[Walker] 後退 x{max(1, int(steps))}")

    def turn_left(self, steps: int = 1):
        with self._lock:
            if not self._ensure_planner():
                return
            for i in range(max(1, int(steps))):
                self._facing_angle += self._TURN_STEP
                fv = self._fv()
                for _ in range(TURN_REPEAT_COUNT):
                    self.send_planner(2, [0, 0, 0], fv)
                    if self._wait_or_stop(TURN_REPEAT_INTERVAL):
                        self.send_planner(0, [0, 0, 0], self._fv())
                        return
                self.send_planner(0, [0, 0, 0], self._fv())
                if i < max(1, int(steps)) - 1 and self._wait_or_stop(0.05):
                    return
        print(f"[Walker] 左旋回 x{max(1, int(steps))} → {np.degrees(self._facing_angle):.0f}°")

    def turn_right(self, steps: int = 1):
        with self._lock:
            if not self._ensure_planner():
                return
            for i in range(max(1, int(steps))):
                self._facing_angle -= self._TURN_STEP
                fv = self._fv()
                for _ in range(TURN_REPEAT_COUNT):
                    self.send_planner(2, [0, 0, 0], fv)
                    if self._wait_or_stop(TURN_REPEAT_INTERVAL):
                        self.send_planner(0, [0, 0, 0], self._fv())
                        return
                self.send_planner(0, [0, 0, 0], self._fv())
                if i < max(1, int(steps)) - 1 and self._wait_or_stop(0.05):
                    return
        print(f"[Walker] 右旋回 x{max(1, int(steps))} → {np.degrees(self._facing_angle):.0f}°")

    def stop(self):
        self._cancel_action(wait=False)
        with self._lock:
            if not self._ensure_planner():
                return
            self.send_planner(0, [0, 0, 0], self._fv())
        print("[Walker] 停止")

    def switch_to_streaming(self):
        self._cancel_action(wait=True)
        with self._lock:
            self.send_command(start=True, stop=False, planner=False)
            self._planner_mode = False
            self._wait_or_stop(0.2)

    def switch_to_planner(self):
        with self._lock:
            self.send_command(start=True, stop=False, planner=True)
            self._planner_mode = True
            self._wait_or_stop(0.5)


# ── インタラクティブ CLI ──────────────────────────────────────

class _RawStdout:
    """raw terminal mode で \n → \r\n に変換するラッパー"""
    def __init__(self, orig):
        self._orig = orig

    def write(self, text):
        self._orig.write(text.replace('\n', '\r\n'))

    def flush(self):
        self._orig.flush()

    def __getattr__(self, name):
        return getattr(self._orig, name)


class InteractiveCLI:
    _HELP = """\
┌──────────────────────────────────────────────────────────────────┐
│  G1 動作テスト  ─  操作方法                                       │
├──────────────────────────┬───────────────────────────────────────┤
│  [raw モード (1キー即時)] │  [コマンド入力 (Enter で開始)]        │
│   w/s/a/d  歩行          │   w[n] s[n] a[n] d[n]  歩行          │
│   Space    停止          │   stop / x       停止                 │
│   1〜9    エントリ再生   │   <番号>         エントリ番号で再生   │
│   n / p   次 / 前        │   <名前>         s1 を再生            │
│   f+数字  強制再生       │   <名前> <s>     例: nod 2             │
│   l       一覧           │                                        │
│   h / ?   ヘルプ         │   n/next  p/prev 次/前                │
│   Ctrl+C  終了           │   f <cmd>        強制実行             │
│                          │   list/l  status  q  ESC              │
└──────────────────────────┴───────────────────────────────────────┘"""

    def __init__(self, entries: list[dict], player: MotionPlayer, walker: WalkerController):
        self._entries  = entries          # フラットリスト
        self._player   = player
        self._walker   = walker
        self._cursor   = 0               # n/p で使うカーソル
        self._running  = True

    # ── 検索 ─────────────────────────────────────────────────

    def _find(self, name: str, sample: Optional[int] = None) -> list[int]:
        """名前部分一致 + sample でエントリインデックスを返す。"""
        result = []
        for i, e in enumerate(self._entries):
            if name.lower() not in e["name"].lower():
                continue
            if sample is not None and e["sample"] != sample:
                continue
            result.append(i)
        return result

    def _default_for_name(self, name: str) -> Optional[int]:
        hits = self._find(name, 1)
        if hits:
            return hits[0]
        hits = self._find(name)
        return hits[0] if hits else None

    # ── 動作再生 ─────────────────────────────────────────────

    def _play_idx(self, idx: int, force: bool = False):
        if not (0 <= idx < len(self._entries)):
            print(f"[Error] 番号が範囲外: {idx + 1}")
            return
        self._cursor = idx
        e   = self._entries[idx]
        tag = "[強制] " if force else ""
        num = idx + 1
        print(f"[動作#{num:3d}] {tag}{e['key']:<30}  {e['desc']}  ({e['dur_sec']:.1f}s)")

        def _run():
            self._walker.switch_to_streaming()
            self._player.play_once(e, force=force)
            while self._player.is_playing():
                time.sleep(0.05)
            self._walker.switch_to_planner()

        threading.Thread(target=_run, daemon=True).start()

    # ── コマンドパーサー ──────────────────────────────────────

    def _handle_command(self, raw: str):
        cmd   = raw.strip()
        if not cmd:
            return
        parts = cmd.split()
        head  = parts[0].lower()

        if head in ('q', 'quit', 'exit'):
            self._running = False
            return
        if head in ('h', 'help', '?'):
            print(self._HELP)
            return
        if head in ('l', 'list'):
            self._print_motion_list()
            return
        if head == 'status':
            st  = "[再生中]" if self._player.is_playing() else "[停止中]"
            cur = self._entries[self._cursor]["key"] if self._entries else "-"
            print(f"Player: {st}  cursor: #{self._cursor+1} {cur}  "
                  f"facing: {np.degrees(self._walker._facing_angle):.0f}°")
            return
        if head in ('stop', 'x'):
            self._player.stop()
            self._walker.stop()
            return
        if head in ('n', 'next'):
            nxt = (self._cursor + 1) % len(self._entries) if self._entries else 0
            self._play_idx(nxt)
            return
        if head in ('p', 'prev'):
            prv = (self._cursor - 1) % len(self._entries) if self._entries else 0
            self._play_idx(prv)
            return

        steps = int(parts[1]) if len(parts) > 1 and parts[1].isdigit() else 1
        if head in ('w', 'forward'):
            self._player.stop()
            n = steps; self._walker.run_action(lambda: self._walker.walk_forward(n))
            return
        if head in ('s', 'backward', 'back'):
            self._player.stop()
            n = steps; self._walker.run_action(lambda: self._walker.walk_backward(n))
            return
        if head in ('a', 'left'):
            self._player.stop()
            n = steps; self._walker.run_action(lambda: self._walker.turn_left(n))
            return
        if head in ('d', 'right'):
            self._player.stop()
            n = steps; self._walker.run_action(lambda: self._walker.turn_right(n))
            return

        force = False
        if head == 'f' and len(parts) > 1:
            force = True
            parts = parts[1:]
            head  = parts[0].lower()

        # 番号指定: "12"
        if head.isdigit():
            self._play_idx(int(head) - 1, force=force)
            return

        # name [sample]: "nod" / "nod 2"
        name = head
        sample_arg = int(parts[1]) if len(parts) > 1 and parts[1].isdigit() else None

        if sample_arg is not None:
            hits = self._find(name, sample_arg)
        else:
            idx = self._default_for_name(name)
            if idx is not None:
                self._play_idx(idx, force=force)
                return
            hits = self._find(name)

        if not hits:
            print(f"[Error] 見つかりません: '{cmd}'")
            return
        if len(hits) == 1 or sample_arg is not None:
            self._play_idx(hits[0], force=force)
        else:
            # 複数候補 → 一覧表示
            print(f"[候補] {len(hits)} 件:")
            for i in hits[:12]:
                e = self._entries[i]
                print(f"  #{i+1:3d}  {e['key']}")

    def _print_motion_list(self):
        if not self._entries:
            print("\n[動作一覧] なし (data/motions/ に .npz を配置してください)\n")
            return
        print(f"\n[動作一覧]  計 {len(self._entries)} エントリ")
        print(f"  {'#':>3}  {'名前':<24} {'S':>2}  {'説明':<24}  秒数")
        print("  " + "─" * 65)
        prev_name = ""
        for i, e in enumerate(self._entries, 1):
            sep = "  " if e["name"] == prev_name else "\n  "
            cur = "▶" if i - 1 == self._cursor else " "
            print(f"{sep}{cur}{i:3d}  {e['name']:<24} {e['sample']:>2}  "
                  f"{e['desc']:<24}  {e['dur_sec']:.1f}s")
            prev_name = e["name"]
        print()

    # ── メインループ ─────────────────────────────────────────

    def run(self):
        print(self._HELP)
        self._print_motion_list()
        print("[raw] WASD=歩行  1〜9=エントリ再生  n/p=次/前  f+数字=強制  Enter=コマンド  Ctrl+C=終了\n")

        fd       = sys.stdin.fileno()
        old_attr = termios.tcgetattr(fd)

        orig_stdout = sys.stdout
        sys.stdout  = _RawStdout(orig_stdout)

        try:
            tty.setraw(fd)
            buf       = ""
            cmd_mode  = False
            f_pending = False

            sys.stdout.write("[raw] > ")
            sys.stdout.flush()

            while self._running:
                if not _select.select([sys.stdin], [], [], 0.1)[0]:
                    continue
                ch = sys.stdin.read(1)

                if cmd_mode:
                    if ch in ('\r', '\n'):
                        sys.stdout.write('\n')
                        sys.stdout.flush()
                        self._handle_command(buf)
                        buf      = ""
                        cmd_mode = False
                        if self._running:
                            sys.stdout.write('\n[raw] > ')
                            sys.stdout.flush()
                    elif ch in ('\x7f', '\x08'):
                        if buf:
                            buf = buf[:-1]
                            sys.stdout.write('\b \b')
                            sys.stdout.flush()
                    elif ch == '\x1b':
                        sys.stdout.write('\n[キャンセル]\n[raw] > ')
                        sys.stdout.flush()
                        buf      = ""
                        cmd_mode = False
                    elif ch == '\x03':
                        self._running = False
                    else:
                        buf += ch
                        sys.stdout.write(ch)
                        sys.stdout.flush()
                    continue

                if ch == '\x03':
                    self._running = False
                    break
                if ch in ('\r', '\n'):
                    f_pending = False
                    cmd_mode  = True
                    sys.stdout.write('\n[cmd] > ')
                    sys.stdout.flush()
                    continue

                if ch.lower() == 'f' and not f_pending:
                    f_pending = True
                    sys.stdout.write('[f]')
                    sys.stdout.flush()
                    continue

                if ch.lower() == 'w':
                    self._player.stop()
                    self._walker.run_action(self._walker.walk_forward)
                    f_pending = False
                elif ch.lower() == 's':
                    self._player.stop()
                    self._walker.run_action(self._walker.walk_backward)
                    f_pending = False
                elif ch.lower() == 'a':
                    self._player.stop()
                    self._walker.run_action(self._walker.turn_left)
                    f_pending = False
                elif ch.lower() == 'd':
                    self._player.stop()
                    self._walker.run_action(self._walker.turn_right)
                    f_pending = False
                elif ch == ' ':
                    self._player.stop()
                    self._walker.stop()
                    f_pending = False
                elif ch.lower() == 'n':
                    nxt = (self._cursor + 1) % len(self._entries) if self._entries else 0
                    self._play_idx(nxt, force=f_pending)
                    f_pending = False
                elif ch.lower() == 'p':
                    prv = (self._cursor - 1) % len(self._entries) if self._entries else 0
                    self._play_idx(prv, force=f_pending)
                    f_pending = False
                elif ch.isdigit() and ch != '0':
                    self._play_idx(int(ch) - 1, force=f_pending)
                    f_pending = False
                elif ch.lower() == 'l':
                    self._print_motion_list()
                    f_pending = False
                elif ch.lower() in ('h', '?'):
                    print(self._HELP)
                    f_pending = False
                else:
                    f_pending = False

        except Exception as e:
            print(f"\n[エラー] {e}", file=orig_stdout)
            import traceback
            traceback.print_exc(file=orig_stdout)
        finally:
            sys.stdout = orig_stdout
            sys.stdout.write('\r\n')
            termios.tcsetattr(fd, termios.TCSADRAIN, old_attr)
            print("終了しました")


# ── エントリポイント ──────────────────────────────────────────

def main():
    p = argparse.ArgumentParser(description="G1 動作テスト CLI")
    p.add_argument("--motion-dir", default=G1_MOTION_DIR,
                   help=f"動作ライブラリルート (デフォルト: {G1_MOTION_DIR})")
    p.add_argument("--zmq-port",  type=int, default=ZMQ_PORT,
                   help=f"ZMQ ポート番号 (デフォルト: {ZMQ_PORT})")
    p.add_argument("--host",      default="localhost",
                   help="ZMQ 送信先ホスト (デフォルト: localhost)")
    args = p.parse_args()

    ctx  = zmq.Context()
    sock = ctx.socket(zmq.PUB)
    sock.bind(f"tcp://*:{args.zmq_port}")
    print(f"[ZMQ] tcp://*:{args.zmq_port}")
    time.sleep(0.5)

    entries = load_motions(args.motion_dir)

    player = MotionPlayer(sock)
    walker = WalkerController(sock)
    walker.start_planner()

    print(f"起動完了  エントリ: {len(entries)} 個  ZMQ: tcp://*:{args.zmq_port}\n")

    def _sigint(sig, frame):
        pass
    signal.signal(signal.SIGINT, _sigint)

    cli = InteractiveCLI(entries, player, walker)
    try:
        cli.run()
    finally:
        player.stop()
        sock.close()
        ctx.term()


if __name__ == "__main__":
    main()
