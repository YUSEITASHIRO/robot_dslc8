#!/usr/bin/env python3
"""
g1_realtime_dialogue.py — G1 ロボット リアルタイム音声対話 + 動作制御
OpenAI Realtime API (日本語) + Kimodo 動作ライブラリ → SONIC ZMQ

動作データは data/motions/<name>/sample_1.npz から読み込みます。
"""

import argparse
import asyncio
import base64
import json
import math
import os
import shutil
import signal
import struct
import sys
import threading
import time
from collections import deque
from typing import Optional

import numpy as np
import zmq

# ── 設定 ──────────────────────────────────────────────────────
OPENAI_API_KEY      = os.environ.get("OPENAI_API_KEY", "")
OPENAI_REALTIME_MODEL = os.environ.get("OPENAI_REALTIME_MODEL", "gpt-4o-realtime-preview")

ZMQ_PORT    = 5556
HEADER_SIZE = 1280
SONIC_FPS   = 50
CHUNK_SIZE  = 5
WALK_STEP_DURATION   = 0.35
TURN_REPEAT_COUNT    = 5       # 運営推奨値（旋回の安定性向上）
TURN_REPEAT_INTERVAL = 0.05    # 運営推奨値（応答性向上）
TURN_STEP_DEG        = 15.0    # 1回の旋回量（度）
TURN_SETTLE_S        = 1.2     # 旋回後の足位置安定待機（秒）
FOOT_REALIGN_S       = 0.35    # 旋回後の微調整歩行（秒）
SETTLE_TICK_S        = 0.05    # 安定待機の刻み幅

METERS_PER_STEP  = 0.35        # 1 WBC ステップあたりの移動距離（m）
GRID_STEP_M      = 1.00        # グリッド間隔（m）
SAFETY_MARGIN    = 0.40        # 壁・障害物からの安全距離（m）

# Phase 4.2: 転倒防止 — ナビゲーション watchdog
NAV_TIMEOUT_S    = 60.0        # navigate_to の最大許容時間（秒）。超過で強制停止

# Phase 4.3: レイテンシ最適化 — Realtime API VAD パラメータ
VAD_THRESHOLD          = 0.5   # 音声検出感度（0.0〜1.0）
VAD_PREFIX_PADDING_MS  = 200   # 発話開始前の余白（ms）。小さいほど早く反応
VAD_SILENCE_DURATION_MS = 500  # 沈黙判定時間（ms）。小さいほど早く応答開始

# Phase 5.1: エコーキャンセル — 音声ゲートパラメータ
ECHO_GATE_COOLDOWN_S = 0.40    # TTS終了後にマイクを再開するまでの待機（秒）

# Phase 5.2: シーンキャッシュ
SCENE_CACHE_TTL_S = 30.0       # look_around 結果のキャッシュ有効期間（秒）

VISION_ENABLED           = os.environ.get("VISION_ENABLED", "false").strip().lower() in {"1", "true", "yes", "on"}
VISION_CAMERA_ZMQ        = os.environ.get("VISION_CAMERA_ZMQ", "tcp://localhost:5555")
VISION_CAMERA            = os.environ.get("VISION_CAMERA", "ego_view")
VISION_DETAIL            = os.environ.get("VISION_DETAIL", "low").strip().lower()
VISION_MAX_STALENESS_SEC = float(os.environ.get("VISION_MAX_STALENESS_SEC", "2.0"))
VISION_SUPPORTED_MODELS  = {"gpt-realtime", "gpt-realtime-2",
                            "gpt-4o-realtime-preview", "gpt-4o-realtime-preview-2024-12-17"}

SAMPLE_RATE   = 24000
MIC_RATE      = 48000
MIC_DEVICE_ID = None  # pyaudio: None=デフォルト or デバイス番号 or デバイス名で部分一致
OUT_DEVICE_ID = None  # pyaudio: None=デフォルト or デバイス番号 or デバイス名で部分一致

REPO_ROOT      = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
G1_MOTION_DIR  = os.path.join(REPO_ROOT, "data", "motions")

SYSTEM_PROMPT = """あなたは携帯電話販売店の「ロボット店長」です。
現在、閉店後に初めてこの店舗を視察に来たエリアマネージャー（部長）と対話しています。

【シチュエーション】
- 部長はほとんど面識のない上司。敬語を使う。
- 部長は「新商品を店の一番奥の通路に置くべきだ」と主張している。
- この店舗は入口正面のガラスからしか外に見えない。奥は死角になる。
- 現在の新商品配置（入口付近）が集客上正しく、奥へ移すと外から見えなくなる。
- スタッフから「阻止してほしい」と頼まれている。
- 評価軸は「円滑なコミュニケーション」（承諾させることではなく、対話の質）。

【行動規則（評価項目に直結）】

■ マルチモーダルなやりとり
- 発話だけで完結させない。うなずき（nod）・相づち（at_ease）など、聞いている姿勢を動作で示す。
- 部長の発話が終わったら、まず動作で反応してから言葉を返す。

■ 対面ならではの話し方
- 場所の説明は言語化しすぎない。「こちらです」と言いながら navigate_to で実際に移動するか、point_at で指し示す。
- 「奥の棚に移すと外から見えなくなります」と説明するより、実際に奥まで歩いて「ここからは外が見えません」と体感させる。

■ 待遇表現とジェスチャーのリンク
- 出迎え → bow_45
- 同意・相槌 → nod または deep_nod
- 軽い否定・辞退 → wave_off
- 強い否定 → cross_arms_x（多用しない）
- 考え中・困惑 → arms_akimbo または shrug
- 誘導 → beckon または this_way_left / this_way_right
- 共感・誠意 → hand_on_chest
- プレゼン・展示 → present_with_both_hands または arms_open

■ 空間を適切に認識した動き
- 移動は navigate_to を使う（BFS で障害物回避済み）。walk_command は直接指示があった場合のみ。
- 誘導するときは先に point_at や beckon で方向を示してから navigate_to を呼ぶ。
- 部長の進路を塞がないよう、先に移動して「こちらにどうぞ」と誘う形を取る。

【店舗レイアウト】
- entrance (0,0)          : 入口（ガラス越しに外から見える・ロボット起動位置）
- new_product (1,0)       : 現在の新商品陳列位置（入口正面・外から丸見え）← 現状の正しい配置
- center (0,3)            : 店内中央
- center_back (0,5)       : 中央奥（外から完全に見えない死角）← 部長が推す場所
- left_shelf (1,3)        : 左棚通路（窓側）
- left_shelf_back (1,5)   : 左棚奥
- right_shelf (-1,3)      : 右棚通路
- right_shelf_back (-1,5) : 右棚奥

【利用可能な動作一覧（select_motion で状況に合う1つを必ず選ぶ）】
あいさつ・礼儀系:
  bow_slight              軽いお辞儀（日常あいさつ）
  bow_45                  45度お辞儀（出迎え・感謝）
  bow_apology             謝罪のお辞儀
  bow_deep                深いお辞儀（強い敬意・謝罪）
  namaste                 合掌（丁寧な挨拶・感謝）
  salute                  敬礼
  handshake_offer         握手を求める仕草
  wave                    手を振る（カジュアルなあいさつ）
  welcome_arms            両腕を広げて迎える

同意・肯定系:
  nod                     うなずき（相槌・同意）
  deep_nod                大きなうなずき（強い同意）
  thumbs_up               親指を立てる（グッドサイン）
  double_thumbs_up        両手でサムズアップ
  fist_pump               こぶしを突き上げる（達成感・喜び）
  clap                    拍手（称賛・喜び）
  banzai                  万歳（大きな喜び）

否定・困惑系:
  wave_off                手を振って断る（軽い否定・「いえいえ」）
  cross_arms_x            腕でバツ印（明確な否定）
  shrug                   肩をすくめる（困惑・わからない）
  lean_back_surprised     後ずさりで驚く

思考・傾聴系:
  lean_forward_interest   前傾みで興味を示す
  hand_on_chest           胸に手を当てる（誠実・共感）
  hands_together_apology  両手を合わせてお願い・謝罪
  arms_akimbo             腰に手を当てる（考え中・ちょっと困惑）
  at_ease                 直立・休めの姿勢（聞いている・待機）
  idle                    自然な待機姿勢

誘導・指示系:
  beckon                  手招き（「こちらへどうぞ」）
  point_forward           前方を指差す
  point_left              左を指差す
  point_right             右を指差す
  point_up                上を指差す
  point_down              下を指差す
  point_to_self           自分を指差す（「わたしが…」）
  point_back_over_shoulder 後ろ・奥を指差す（「あちらに…」）
  this_way_left           左へ誘導（「左へどうぞ」）
  this_way_right          右へ誘導（「右へどうぞ」）
  present_with_both_hands 両手で何かを示す（プレゼン・展示）
  arms_open               両腕を開く（「ご覧ください」「歓迎」）
  halt                    手のひらを向けて止める（「少々お待ちを」）

【Function Call の使い方（必須ルール）】
- 返答前に必ず select_motion を1回呼ぶ（navigate_to・walk_command 時は省略可）
- 場所を示す場面では navigate_to で実際に移動してから発話する
- 直接的な移動依頼（「前進して」等）のみ walk_command を使う
- navigate_to と walk_command は select_motion の代わり（同時呼び出し不可）
- 返答は2〜3文程度の簡潔さを保つ
"""

# ── 店舗グリッド座標（座標系は g1_move_test.py 準拠） ────────
# +x = 前方(ユーザー・左壁方向), +y = 左(店内奥方向)
# x=1: 左側通路, x=0: 中央, x=-1: 右側通路
# y=0: 入口付近, y=6: 奥限界

_NAMED_LOCATIONS: dict[str, tuple] = {
    "entrance":         ( 0,  0),
    "new_product":      ( 1,  0),
    "center":           ( 0,  3),
    "center_back":      ( 0,  5),
    "left_shelf":       ( 1,  3),
    "left_shelf_back":  ( 1,  5),
    "right_shelf":      (-1,  3),
    "right_shelf_back": (-1,  5),
}

_FACING_ANGLES: dict[str, float] = {
    "user":     0.0,    # ユーザー・左壁方向（+x, 0°）
    "entrance": 0.0,    # 入口方向（ユーザーと同じ）
    "counter":  90.0,   # 奥・カウンター方向（+y, 90°）
    "back":     90.0,   # 店内奥方向
}

# ── 店舗ジオメトリ（g1_move_test.py 準拠） ─────────────────────
_WALLS = [
    ( 0.0,  9.0,  2.0,  0.05),
    ( 0.0, -1.0,  2.0,  0.05),
    (-2.0,  4.0,  0.05, 5.05),
    ( 2.0,  5.0,  0.05, 4.0 ),
]
_OBSTACLES = [
    ( 1.78, 7.5,  0.2,  1.5  ),
    ( 1.78, 3.5,  0.2,  2.5  ),
    (-1.78, 7.5,  0.2,  1.5  ),
    (-1.78, 3.5,  0.2,  2.5  ),
    (-1.78, 0.0,  0.2,  1.0  ),
    ( 1.5,  1.0,  0.5,  0.05 ),
    ( 1.5,  1.25, 0.4,  0.2  ),
    (-0.5,  7.8,  0.9,  0.7  ),
    ( 0.65, 7.8,  0.2,  0.65 ),
]
_ALL_BOXES = _WALLS + _OBSTACLES


def _is_safe(x: float, y: float) -> bool:
    for cx, cy, hdx, hdy in _ALL_BOXES:
        if abs(x - cx) < hdx + SAFETY_MARGIN and abs(y - cy) < hdy + SAFETY_MARGIN:
            return False
    return True


def build_grid() -> dict:
    step = GRID_STEP_M
    xi_min = math.ceil((-2.0 + SAFETY_MARGIN) / step)
    xi_max = math.floor(( 2.0 - SAFETY_MARGIN) / step)
    yi_min = math.ceil((-1.0 + SAFETY_MARGIN) / step)
    yi_max = math.floor(( 9.0 - SAFETY_MARGIN) / step)
    grid: dict = {}
    for xi in range(xi_min, xi_max + 1):
        for yi in range(yi_min, yi_max + 1):
            x = round(xi * step, 6)
            y = round(yi * step, 6)
            if _is_safe(x, y):
                grid[(xi, yi)] = (x, y)
    return grid


_DIRS4 = [(1, 0), (-1, 0), (0, 1), (0, -1)]


def _bfs_path(grid: dict, start: tuple, end: tuple) -> Optional[list]:
    if start == end:
        return [start]
    q: deque = deque([(start, [start])])
    visited = {start}
    while q:
        pos, path = q.popleft()
        for dx, dy in _DIRS4:
            nxt = (pos[0] + dx, pos[1] + dy)
            if nxt == end:
                return path + [nxt]
            if nxt not in visited and nxt in grid:
                visited.add(nxt)
                q.append((nxt, path + [nxt]))
    return None


def _group_path(path: list) -> list:
    if len(path) < 2:
        return []
    segs = []
    cd = (path[1][0] - path[0][0], path[1][1] - path[0][1])
    cnt = 1
    for i in range(2, len(path)):
        d = (path[i][0] - path[i-1][0], path[i][1] - path[i-1][1])
        if d == cd:
            cnt += 1
        else:
            segs.append((cd, cnt))
            cd, cnt = d, 1
    segs.append((cd, cnt))
    return segs


def _plan_path(grid: dict, start: tuple, end: tuple) -> Optional[list]:
    if start == end:
        return []
    xi0, yi0 = start
    xi1, yi1 = end
    dx = (1 if xi1 > xi0 else -1) if xi1 != xi0 else 0
    dy = (1 if yi1 > yi0 else -1) if yi1 != yi0 else 0
    steps_x, steps_y = abs(xi1 - xi0), abs(yi1 - yi0)

    def clear(ox, oy, ddx, ddy, n):
        return all((ox + ddx*i, oy + ddy*i) in grid for i in range(1, n+1))

    if (steps_x == 0 or clear(xi0, yi0, dx, 0, steps_x)) and \
       (steps_y == 0 or clear(xi1, yi0, 0, dy, steps_y)):
        segs = []
        if steps_x: segs.append(((dx, 0), steps_x))
        if steps_y: segs.append(((0, dy), steps_y))
        return segs
    if (steps_y == 0 or clear(xi0, yi0, 0, dy, steps_y)) and \
       (steps_x == 0 or clear(xi0, yi1, dx, 0, steps_x)):
        segs = []
        if steps_y: segs.append(((0, dy), steps_y))
        if steps_x: segs.append(((dx, 0), steps_x))
        return segs
    path = _bfs_path(grid, start, end)
    return _group_path(path) if path else None


def _nearest_grid(grid: dict, x: float, y: float) -> Optional[tuple]:
    best, best_d = None, math.inf
    for (xi, yi), (gx, gy) in grid.items():
        d = math.hypot(x - gx, y - gy)
        if d < best_d:
            best_d, best = d, (xi, yi)
    return best


def _norm_angle(a: float) -> float:
    while a >  math.pi: a -= 2 * math.pi
    while a <= -math.pi: a += 2 * math.pi
    return a


def _dir_to_sonic(dx: int, dy: int) -> float:
    return math.atan2(dy, dx)


def _pointing_motion(robot_xi: int, robot_yi: int, facing_rad: float,
                     target_xi: int, target_yi: int) -> str:
    """ロボットの現在位置・向きと対象物の位置から適切な指さしモーション名を返す。"""
    dx = (target_xi - robot_xi) * GRID_STEP_M
    dy = (target_yi - robot_yi) * GRID_STEP_M
    if abs(dx) < 0.01 and abs(dy) < 0.01:
        return "point_forward"
    target_rad = math.atan2(dy, dx)
    rel_deg = math.degrees(_norm_angle(target_rad - facing_rad))
    if abs(rel_deg) < 45:
        return "point_forward"
    elif 45 <= rel_deg < 135:
        return "this_way_left"
    elif -135 < rel_deg <= -45:
        return "this_way_right"
    else:
        return "point_back_over_shoulder"


def _normalize_device_selector(value):
    if value is None:
        return None
    if isinstance(value, int):
        return value
    text = str(value).strip()
    if not text:
        return None
    if text.isdigit():
        return int(text)
    return text


def _restart_pipewire_services_if_available() -> bool:
    """Restart PipeWire user services when running on Linux with systemd."""
    if not sys.platform.startswith("linux"):
        return False

    if shutil.which("systemctl") is None:
        return False

    import subprocess

    subprocess.run(
        ["systemctl", "--user", "restart", "pipewire", "pipewire-pulse", "wireplumber"],
        check=False,
        capture_output=True,
    )
    return True


def _get_default_device_index(pa, is_input: bool) -> Optional[int]:
    try:
        info = pa.get_default_input_device_info() if is_input else pa.get_default_output_device_info()
        return int(info["index"])
    except Exception:
        return None


def resolve_audio_device(pa, selector, is_input: bool, purpose: str) -> Optional[int]:
    selector = _normalize_device_selector(selector)
    channel_key = "maxInputChannels" if is_input else "maxOutputChannels"
    kind = "input" if is_input else "output"

    if isinstance(selector, int):
        info = pa.get_device_info_by_index(selector)
        if info[channel_key] <= 0:
            raise RuntimeError(f"[{purpose}] device={selector} は {kind} デバイスではありません")
        return selector

    if isinstance(selector, str):
        target = selector.lower()
        for i in range(pa.get_device_count()):
            info = pa.get_device_info_by_index(i)
            if info[channel_key] > 0 and target in info["name"].lower():
                return i
        raise RuntimeError(f"[{purpose}] '{selector}' に一致する {kind} デバイスが見つかりません")

    return _get_default_device_index(pa, is_input)


def print_audio_devices():
    import pyaudio

    pa = pyaudio.PyAudio()
    try:
        print("[Audio] 利用可能なデバイス一覧")
        for i in range(pa.get_device_count()):
            info = pa.get_device_info_by_index(i)
            ins = int(info.get("maxInputChannels", 0))
            outs = int(info.get("maxOutputChannels", 0))
            print(f"  {i:2d}: {info['name']}  (in={ins}, out={outs})")
    finally:
        pa.terminate()




# ── Kimodo 動作ライブラリ読み込み ──────────────────────────────

def _compute_jv(jp: np.ndarray) -> np.ndarray:
    jv = np.zeros_like(jp)
    if len(jp) > 1:
        jv[:-1] = (jp[1:] - jp[:-1]) * SONIC_FPS
        jv[-1] = jv[-2]
    return jv


def load_motions(motion_dir: str = G1_MOTION_DIR) -> dict:
    """data/motions/<name>/sample_1.npz を読み込む。
    戻り値は {name: {jp, jv, bq, desc, dur}} の辞書。
    """
    from pathlib import Path
    root = Path(motion_dir)
    if not root.exists():
        print(f"[Motion] ディレクトリが見つかりません: {motion_dir}")
        return {}

    available: dict = {}
    for name_dir in sorted(root.iterdir()):
        if not name_dir.is_dir():
            continue
        name = name_dir.name
        npz_path = name_dir / "sample_1.npz"
        if not npz_path.exists():
            print(f"[Motion] - {name}  (未生成)")
            continue
        try:
            d  = np.load(npz_path)
            jp = d["jp"].astype(np.float32)
            jv = (d["jv"].astype(np.float32) if "jv" in d else _compute_jv(jp))
            bq = (d["bq"].astype(np.float32) if "bq" in d
                  else np.tile([1., 0., 0., 0.], (len(jp), 1)).astype(np.float32))
            dur_sec = len(jp) / SONIC_FPS
            rel = npz_path.relative_to(root.parent)
            available[name] = {"jp": jp, "jv": jv, "bq": bq,
                                "desc": name, "dur": dur_sec}
            print(f"[Motion] ✓ {name:<30} ({rel})  {dur_sec:.1f}s")
        except Exception as e:
            print(f"[Motion] ✗ {name}  ({e})")
    print(f"\n[Motion] 利用可能: {len(available)} 個\n")
    return available


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


# ── 動作プレイヤー ────────────────────────────────────────────

class MotionPlayer:
    def __init__(self, sock):
        self._sock   = sock
        self._thread: Optional[threading.Thread] = None
        self._stop   = threading.Event()
        self._fi     = 0  # フレームカウンター

    def is_playing(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def play_once(self, motion: dict, force: bool = False):
        """動作を1回再生。再生中なら無視（force=True の場合は中断して開始）"""
        if not force and self.is_playing():
            return  # 再生中は無視
        self._stop.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=2.0)
        self._stop.clear()
        self._thread = threading.Thread(
            target=self._run, args=(motion,), daemon=True)
        self._thread.start()

    def stop(self):
        self._stop.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=2.0)

    def _run(self, motion: dict):
        """bones_to_sonic.py と完全同一の送信ロジック"""
        jp = motion["jp"]
        jv = motion["jv"]
        bq = motion["bq"]
        T  = len(jp)

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
        # 動作完了 → WBC が自動的に引き継ぐ（bones_to_sonic.py と同じ）



# ── 歩行コントローラー ────────────────────────────────────────

class WalkerController:
    def __init__(self, sock):
        self._sock         = sock
        self._facing_angle = 0.0
        self._TURN_STEP    = math.radians(TURN_STEP_DEG)
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
        """Return True if an in-flight action was cancelled."""
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
                self.send_planner(0, [0,0,0], self._fv())
                self._planner_mode = True
                print("[Walker] planner モード開始")

    def _ensure_planner(self):
        if not self._planner_mode:
            self.send_command(start=True, stop=False, planner=True)
            if self._wait_or_stop(0.3):
                return False
            self._planner_mode = True
        return True

    def _walk_linear(self, sign: float, steps: int = 1, step_duration: float = WALK_STEP_DURATION):
        steps = max(1, int(steps))
        with self._lock:
            if not self._ensure_planner():
                return False
            for step_index in range(steps):
                a = self._facing_angle
                mv = [sign * np.cos(a), sign * np.sin(a), 0.0]
                self.send_planner(2, mv, self._fv())
                if self._wait_or_stop(step_duration):
                    self.send_planner(0, [0,0,0], self._fv())
                    return False
                self.send_planner(0, [0,0,0], self._fv())
                if step_index < steps - 1 and self._wait_or_stop(0.08):
                    return False
        return True

    def walk_forward(self, steps: int = 1):
        if self._walk_linear(1.0, steps=steps):
            print(f"[Walker] 前進 x{max(1, int(steps))}")

    def walk_backward(self, steps: int = 1):
        if self._walk_linear(-1.0, steps=steps):
            print(f"[Walker] 後退 x{max(1, int(steps))}")

    def turn_left(self, steps: int = 1):
        with self._lock:
            if not self._ensure_planner():
                return False
            for step_index in range(max(1, int(steps))):
                self._facing_angle += self._TURN_STEP
                fv = self._fv()
                for _ in range(TURN_REPEAT_COUNT):
                    self.send_planner(2, [0,0,0], fv)
                    if self._wait_or_stop(TURN_REPEAT_INTERVAL):
                        self.send_planner(0, [0,0,0], self._fv())
                        return False
                self.send_planner(0, [0,0,0], self._fv())
                if step_index < max(1, int(steps)) - 1 and self._wait_or_stop(0.05):
                    return False
        print(f"[Walker] 左旋回 x{max(1, int(steps))} → {np.degrees(self._facing_angle):.0f}°")
        return True

    def turn_right(self, steps: int = 1):
        with self._lock:
            if not self._ensure_planner():
                return False
            for step_index in range(max(1, int(steps))):
                self._facing_angle -= self._TURN_STEP
                fv = self._fv()
                for _ in range(TURN_REPEAT_COUNT):
                    self.send_planner(2, [0,0,0], fv)
                    if self._wait_or_stop(TURN_REPEAT_INTERVAL):
                        self.send_planner(0, [0,0,0], self._fv())
                        return False
                self.send_planner(0, [0,0,0], self._fv())
                if step_index < max(1, int(steps)) - 1 and self._wait_or_stop(0.05):
                    return False
        print(f"[Walker] 右旋回 x{max(1, int(steps))} → {np.degrees(self._facing_angle):.0f}°")
        return True

    def stop(self):
        self._cancel_action(wait=False)
        with self._lock:
            if not self._ensure_planner():
                return
            self.send_planner(0, [0,0,0], self._fv())
        print("[Walker] 停止")

    def settle_facing(self, settle_s: float = TURN_SETTLE_S) -> bool:
        """旋回後、mode=2 ゼロ移動で足位置を安定させる。"""
        fv = self._fv()
        n  = max(1, int(settle_s / SETTLE_TICK_S))
        for _ in range(n):
            self.send_planner(2, [0.0, 0.0, 0.0], fv)
            if self._wait_or_stop(SETTLE_TICK_S):
                self.send_planner(0, [0, 0, 0], fv)
                return False
        self.send_planner(0, [0, 0, 0], fv)
        return True

    def micro_align(self) -> bool:
        """旋回直後に短い歩行で両足を揃える。"""
        fv = self._fv()
        self.send_planner(2, fv, fv)
        if self._wait_or_stop(FOOT_REALIGN_S):
            self.send_planner(0, [0, 0, 0], fv)
            return False
        self.send_planner(0, [0, 0, 0], fv)
        return True

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


# ── グリッドナビゲーター ──────────────────────────────────────

class GridNavigator:
    """店舗グリッド上での自律ナビゲーション。"""

    def __init__(self, walker: WalkerController, grid: dict,
                 start_xi: int = 0, start_yi: int = 0):
        self._walker = walker
        self._grid   = grid
        self._xi     = start_xi
        self._yi     = start_yi
        walker._facing_angle = 0.0  # 起動時はユーザー方向（+x = 0°）

    @property
    def cell(self) -> tuple:
        return (self._xi, self._yi)

    @property
    def world_pos(self) -> tuple:
        return self._grid.get((self._xi, self._yi),
                              (round(self._xi * GRID_STEP_M, 3),
                               round(self._yi * GRID_STEP_M, 3)))

    def face_direction(self, target_deg: float) -> bool:
        """その場で指定角度（度）に向きを変える。"""
        target_rad = math.radians(target_deg)
        w = self._walker
        delta = _norm_angle(target_rad - w._facing_angle)
        n = round(abs(delta) / math.radians(TURN_STEP_DEG))
        if n == 0:
            return True
        turn_fn = w.turn_left if delta >= 0 else w.turn_right
        if not turn_fn(n):
            return False
        if not w.settle_facing():
            return False
        return w.micro_align()

    def goto(self, xi: int, yi: int,
             facing_deg: Optional[float] = None) -> bool:
        """指定グリッドセルへ移動し、オプションで向きを変える。"""
        if (xi, yi) not in self._grid:
            print(f"[Nav] ({xi},{yi}) はグリッド外または障害物内")
            return False

        if (xi, yi) != (self._xi, self._yi):
            segs = _plan_path(self._grid, (self._xi, self._yi), (xi, yi))
            if segs is None:
                print(f"[Nav] 経路なし: ({self._xi},{self._yi}) → ({xi},{yi})")
                return False

            total = sum(c for _, c in segs)
            tx, ty = self._grid[(xi, yi)]
            print(f"[Nav] → ({xi},{yi}) ({tx:.2f},{ty:.2f})  {total*GRID_STEP_M:.2f}m  セグ数={len(segs)}")

            if not self._execute(segs):
                return False
        else:
            wx, wy = self.world_pos
            print(f"[Nav] 既にその位置 ({xi},{yi}) ({wx:.2f},{wy:.2f})")

        if facing_deg is not None:
            print(f"[Nav] 向き変更 → {facing_deg:.0f}°")
            return self.face_direction(facing_deg)

        return True

    def _execute(self, segs: list) -> bool:
        w = self._walker
        for seg_idx, ((dx, dy), cnt) in enumerate(segs):
            travel_sonic = _dir_to_sonic(dx, dy)
            delta = _norm_angle(travel_sonic - w._facing_angle)
            n     = round(abs(delta) / math.radians(TURN_STEP_DEG))
            if n > 0:
                turn_fn = w.turn_left if delta >= 0 else w.turn_right
                if not turn_fn(n):
                    return False
                if not w.settle_facing():
                    return False

            wbc_steps = max(1, round(cnt * GRID_STEP_M / METERS_PER_STEP))
            print(f"  [{seg_idx+1}/{len(segs)}] 前進 {cnt}格子 → {wbc_steps}ステップ")
            if not w.walk_forward(wbc_steps):
                return False

            self._xi += dx * cnt
            self._yi += dy * cnt

        w.settle_facing(settle_s=0.5)
        wx, wy = self.world_pos
        print(f"  ✓ 到着 ({self._xi},{self._yi}) ({wx:.2f},{wy:.2f})")
        return True


# ── カメラサブスクライバー（Phase 3.1 → 運営提供アーキテクチャに移行） ──────

class CameraFrameSubscriber:
    """ZMQ カメラストリームを購読し、鮮度チェック付きで最新フレームを提供する。

    運営提供の CameraFrameSubscriber をベースに look_around 互換 API を追加。
    """

    def __init__(self, zmq_url: str, camera: str,
                 max_staleness_sec: float, detail: str):
        self.zmq_url          = zmq_url
        self.camera           = camera
        self.max_staleness_sec = max_staleness_sec
        self.detail           = detail if detail in {"low", "auto", "high"} else "low"
        self._stop            = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._lock            = threading.Lock()
        self._latest          = None
        self._fallback_warned = False
        self._missing_warned  = False

    def start(self):
        import importlib.util
        if importlib.util.find_spec("msgpack") is None:
            print("[Vision] msgpack が見つかりません。pip install msgpack")
            return
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        print(f"[Vision] カメラ購読: {self.zmq_url}  camera={self.camera}")

    def stop(self):
        self._stop.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.0)

    def latest_image(self):
        """最新フレームを {"camera", "image_url", "timestamp", "age"} で返す。未受信 or 古い場合は None。"""
        with self._lock:
            latest = self._latest.copy() if self._latest else None
        if not latest:
            if not self._missing_warned:
                print("[Vision] まだカメラ画像を受信していません。")
                self._missing_warned = True
            return None
        age = time.time() - latest["timestamp"]
        if age > self.max_staleness_sec:
            print(f"[Vision] カメラ画像が古いです ({age:.1f}s)。スキップします。")
            return None
        latest["age"] = age
        return latest

    def get_latest_b64(self) -> Optional[str]:
        """look_around 互換: base64 JPEG 文字列を返す（data: プレフィックスなし）。"""
        frame = self.latest_image()
        if frame is None:
            return None
        url = frame["image_url"]
        prefix = "data:image/jpeg;base64,"
        return url[len(prefix):] if url.startswith(prefix) else url

    def _run(self):
        import msgpack

        ctx = zmq.Context()
        sock = ctx.socket(zmq.SUB)
        sock.setsockopt(zmq.SUBSCRIBE, b"")
        sock.setsockopt(zmq.CONFLATE, 1)
        sock.setsockopt(zmq.RCVTIMEO, 200)
        sock.setsockopt(zmq.LINGER, 0)
        sock.connect(self.zmq_url)
        try:
            while not self._stop.is_set():
                try:
                    raw = sock.recv()
                except zmq.Again:
                    continue
                try:
                    payload = msgpack.unpackb(raw, raw=False)
                except Exception as e:
                    print(f"[Vision] payload decode 失敗: {e}")
                    continue

                images = payload.get("images", {}) if isinstance(payload, dict) else {}
                if not images:
                    continue

                camera = self.camera if self.camera in images else next(iter(images))
                if camera != self.camera and not self._fallback_warned:
                    print(f"[Vision] camera={self.camera} がないため {camera} を使用します")
                    self._fallback_warned = True

                encoded = images.get(camera)
                if not encoded:
                    continue
                if isinstance(encoded, bytes):
                    encoded = encoded.decode("utf-8")

                with self._lock:
                    self._latest = {
                        "camera":    camera,
                        "image_url": f"data:image/jpeg;base64,{encoded}",
                        "timestamp": time.time(),
                    }
                    self._missing_warned = False
        finally:
            sock.close(0)
            ctx.term()


# ── キーボード手動制御 ────────────────────────────────────────

class KeyboardController:
    def __init__(self, walker, player):
        self._walker  = walker
        self._player  = player
        self._running = False
        self._thread: Optional[threading.Thread] = None

    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.0)

    def _loop(self):
        if not sys.platform.startswith("linux"):
            print("[Keyboard] WASD 手動制御は Linux のみ対応しています。スキップします。")
            return
        import tty, termios, select
        fd  = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        print("[Keyboard] WASD 手動制御有効 (W=前進 S=後退 A=左旋回 D=右旋回 Space=停止)")
        try:
            tty.setraw(fd)
            while self._running:
                if not select.select([sys.stdin], [], [], 0.1)[0]:
                    continue
                key = sys.stdin.read(1).lower()
                if key == "\x03":
                    self._running = False
                    os.kill(os.getpid(), signal.SIGINT)
                    return
                if key == "w":
                    self._player.stop()
                    self._walker.run_action(self._walker.walk_forward)
                elif key == "s":
                    self._player.stop()
                    self._walker.run_action(self._walker.walk_backward)
                elif key == "a":
                    self._player.stop()
                    self._walker.run_action(self._walker.turn_left)
                elif key == "d":
                    self._player.stop()
                    self._walker.run_action(self._walker.turn_right)
                elif key == " ":
                    self._player.stop()
                    self._walker.stop()
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)


# ── Realtime API ──────────────────────────────────────────────

class RealtimeDialogue:
    URL = f"wss://api.openai.com/v1/realtime?model={OPENAI_REALTIME_MODEL}"

    def __init__(
        self,
        motions: dict,
        player: MotionPlayer,
        walker,
        navigator: Optional["GridNavigator"] = None,
        camera: Optional["CameraFrameSubscriber"] = None,
        vad: bool = True,
        mic_device=None,
        out_device=None,
    ):
        self.motions   = motions
        self.player    = player
        self.walker    = walker
        self.navigator = navigator
        self.camera    = camera
        self.vad       = vad
        self.mic_device = _normalize_device_selector(mic_device)
        self.out_device = _normalize_device_selector(out_device)
        self._ws      = None
        self._abuf    = bytearray()
        self._nav_lock         = threading.Lock()
        self._backchannel_timer: Optional[threading.Timer] = None
        self._speaking_until: float = 0.0   # Phase 5.1: TTS再生中はここまでマイクをゲート
        self._scene_cache: Optional[str] = None          # Phase 5.2: Vision解析キャッシュ
        self._scene_cache_ts: float = 0.0               # Phase 5.2: キャッシュ取得時刻

    # ── Phase 3.2: バックチャネル相づち ──────────────────────────

    def _schedule_backchannel(self, delay: float = 1.5):
        """発話中の相づち（nod）を delay 秒後に再生する。"""
        self._cancel_backchannel()
        player = self.player
        walker = self.walker
        motions = self.motions

        def _do_nod():
            if "nod" not in motions:
                return
            walker.switch_to_streaming()
            player.play_once(motions["nod"], force=False)
            def _restore():
                while player.is_playing():
                    time.sleep(0.05)
                walker.switch_to_planner()
            threading.Thread(target=_restore, daemon=True).start()

        self._backchannel_timer = threading.Timer(delay, _do_nod)
        self._backchannel_timer.daemon = True
        self._backchannel_timer.start()

    def _cancel_backchannel(self):
        if self._backchannel_timer is not None:
            self._backchannel_timer.cancel()
            self._backchannel_timer = None

    # ── Phase 3.1: カメラ映像解析 ─────────────────────────────

    async def _analyze_scene(self) -> str:
        """最新カメラ映像を GPT-4o Vision で解析してシーン説明を返す。"""
        if self.camera is None:
            return "カメラ未接続"
        # Phase 5.2: キャッシュが有効なら再利用（Vision API 節約）
        if self._scene_cache and (time.monotonic() - self._scene_cache_ts) < SCENE_CACHE_TTL_S:
            return self._scene_cache
        b64 = self.camera.get_latest_b64()
        if b64 is None:
            return "映像未受信（シミュレーターが起動しているか確認してください）"

        payload = {
            "model": "gpt-4o",
            "messages": [
                {
                    "role": "user",
                    "content": [
                        {
                            "type": "image_url",
                            "image_url": {"url": f"data:image/jpeg;base64,{b64}"},
                        },
                        {
                            "type": "text",
                            "text": (
                                "これはロボット店長の一人称視点カメラ映像です。"
                                "以下を簡潔な日本語で答えてください（全体2〜4文）:\n"
                                "1. 人物（部長）の位置（正面/左/右、近い/遠い）\n"
                                "2. 人物の向き（こちらを向いているか）\n"
                                "3. 周囲の障害物や特記事項"
                            ),
                        },
                    ],
                }
            ],
            "max_tokens": 200,
        }
        import urllib.request
        data = json.dumps(payload).encode()
        req = urllib.request.Request(
            "https://api.openai.com/v1/chat/completions",
            data=data,
            headers={
                "Authorization": f"Bearer {OPENAI_API_KEY}",
                "Content-Type": "application/json",
            },
        )
        try:
            loop = asyncio.get_event_loop()
            def _call():
                with urllib.request.urlopen(req, timeout=10) as resp:
                    return json.loads(resp.read())
            result = await loop.run_in_executor(None, _call)
            desc = result["choices"][0]["message"]["content"]
            self._scene_cache = desc          # Phase 5.2: キャッシュ更新
            self._scene_cache_ts = time.monotonic()
            return desc
        except Exception as e:
            return f"解析エラー: {e}"

    async def _initial_scene_scan(self):
        """接続後に初回シーンスキャンを実施してキャッシュを温める。"""
        await asyncio.sleep(3.0)
        if self.camera:
            print("[Vision] 初回シーンスキャン中...")
            desc = await self._analyze_scene()
            print(f"[Vision] 初回スキャン完了: {desc[:60]}...")

    async def _submit_scene_analysis(self, call_id: str):
        """look_around の結果（Vision解析）を Realtime API に返す。"""
        desc = await self._analyze_scene()
        print(f"\n[Vision] {desc}")
        await self._send({
            "type": "conversation.item.create",
            "item": {
                "type": "function_call_output",
                "call_id": call_id,
                "output": desc,
            },
        })
        await self._send({"type": "response.create"})

    async def connect(self):
        try:
            import websockets
        except ImportError:
            print("pip install websockets"); sys.exit(1)

        print(f"[Realtime] モデル: {OPENAI_REALTIME_MODEL}")
        self._ws = await websockets.connect(
            self.URL,
            additional_headers={
                "Authorization": f"Bearer {OPENAI_API_KEY}",
            },
            max_size=10 * 1024 * 1024,
        )
        print("[Realtime] 接続しました")

        tool_motion = {
            "type": "function",
            "name": "select_motion",
            "description": "会話の中で自然なタイミングで呼び出す。挨拶・感情表現・強調場面など、3〜5回に1回程度の頻度で会話の雰囲気に合った動作を選択する。",
            "parameters": {
                "type": "object",
                "properties": {
                    "motion_name": {
                        "type": "string",
                        "enum": list(self.motions.keys()),
                        "description": "動作名",
                    }
                },
                "required": ["motion_name"],
            },
        }
        tool_walk = {
            "type": "function",
            "name": "walk_command",
            "description": "ロボットを移動させる。歩数や旋回回数が分かる場合は steps に入れる。",
            "parameters": {
                "type": "object",
                "properties": {
                    "direction": {
                        "type": "string",
                        "enum": ["forward", "backward", "turn_left", "turn_right", "stop"],
                    },
                    "steps": {
                        "type": "integer",
                        "minimum": 1,
                        "maximum": 10,
                        "description": "前進・後退の歩数、または左右旋回の回数。省略時は1。",
                    },
                },
                "required": ["direction"],
            },
        }
        tool_navigate = {
            "type": "function",
            "name": "navigate_to",
            "description": (
                "ロボットを店舗内の名前付き場所へ自律移動させる。"
                "「こちらの棚をご覧ください」など現場で説明する際に使用する。"
                "navigate_to を使う場合は select_motion や walk_command を同時に呼ばない。"
            ),
            "parameters": {
                "type": "object",
                "properties": {
                    "location": {
                        "type": "string",
                        "enum": list(_NAMED_LOCATIONS.keys()),
                        "description": (
                            "移動先の名前: "
                            "entrance=入口, new_product=現在の新商品位置(入口正面), "
                            "center=店内中央, center_back=中央奥(死角エリア), "
                            "left_shelf=左棚通路, left_shelf_back=左棚奥, "
                            "right_shelf=右棚通路, right_shelf_back=右棚奥"
                        ),
                    },
                    "facing": {
                        "type": "string",
                        "enum": list(_FACING_ANGLES.keys()),
                        "description": (
                            "到着後の向き: "
                            "user/entrance=入口・ユーザー方向, "
                            "counter/back=奥・カウンター方向"
                        ),
                    },
                },
                "required": ["location"],
            },
        }

        tool_point_at = {
            "type": "function",
            "name": "point_at",
            "description": (
                "特定の場所を指差す動作を再生する。「こちらの棚が…」「あちらに…」など"
                "空間を指示する発話と連動して呼び出す。ロボットの現在位置を考慮して"
                "適切な方向の指差しモーションを自動選択する。"
                "select_motion の代わりとして使用し、同時に呼ばない。"
            ),
            "parameters": {
                "type": "object",
                "properties": {
                    "location": {
                        "type": "string",
                        "enum": list(_NAMED_LOCATIONS.keys()),
                        "description": "指差す場所の名前（_NAMED_LOCATIONS のキー）",
                    }
                },
                "required": ["location"],
            },
        }
        tool_look_around = {
            "type": "function",
            "name": "look_around",
            "description": (
                "ロボットの一人称カメラ映像を解析し、部長の位置・向き・"
                "周囲の状況を把握する。対話の冒頭や、部長の位置を確認したいときに呼ぶ。"
            ),
            "parameters": {
                "type": "object",
                "properties": {},
                "required": [],
            },
        }

        cfg = {
            "modalities": ["text", "audio"],
            "instructions": SYSTEM_PROMPT,
            "voice": "shimmer",
            "input_audio_format": "pcm16",
            "output_audio_format": "pcm16",
            "input_audio_transcription": {"model": "whisper-1"},
            "tools": [tool_motion, tool_walk, tool_navigate, tool_point_at, tool_look_around],
            "tool_choice": "auto",
        }
        if self.vad:
            # Phase 4.3: VAD パラメータをトップレベル定数で管理（レイテンシ最適化）
            cfg["turn_detection"] = {
                "type": "server_vad",
                "threshold": VAD_THRESHOLD,
                "prefix_padding_ms": VAD_PREFIX_PADDING_MS,
                "silence_duration_ms": VAD_SILENCE_DURATION_MS,
            }
        await self._ws.send(json.dumps({"type": "session.update", "session": cfg}))
        asyncio.create_task(self._initial_scene_scan())  # Phase 5.2: 接続後にキャッシュを温める

    async def _send(self, msg):
        if self._ws:
            await self._ws.send(json.dumps(msg))

    async def stream_mic(self):
        import pyaudio
        loop = asyncio.get_event_loop()
        q: asyncio.Queue = asyncio.Queue()

        # PipeWire を再起動して音声デバイスを再認識
        if _restart_pipewire_services_if_available():
            await asyncio.sleep(3)

        pa = pyaudio.PyAudio()
        dev_index = resolve_audio_device(pa, self.mic_device, is_input=True, purpose="マイク")
        if dev_index is None:
            pa.terminate()
            raise RuntimeError("[マイク] 入力デバイスが見つかりません")
        info = pa.get_device_info_by_index(dev_index)
        print(f"[マイク] {info['name']} device={dev_index}, 48000Hz → 24000Hz")

        def cb(in_data, frame_count, time_info, status):
            pcm = np.frombuffer(in_data, dtype=np.int16)
            down = pcm[::2]  # 48000 → 24000
            loop.call_soon_threadsafe(q.put_nowait, down.tobytes())
            return (None, pyaudio.paContinue)

        stream = pa.open(
            format=pyaudio.paInt16,
            channels=1,
            rate=48000,
            input=True,
            input_device_index=dev_index,
            frames_per_buffer=2048,
            stream_callback=cb,
        )
        stream.start_stream()
        try:
            while True:
                pcm = await q.get()
                if time.monotonic() < self._speaking_until:
                    continue  # Phase 5.1: TTS再生中はマイクをゲート（エコーキャンセル）
                await self._send({
                    "type": "input_audio_buffer.append",
                    "audio": base64.b64encode(pcm).decode(),
                })
        finally:
            stream.stop_stream()
            stream.close()
            pa.terminate()

    async def play_audio(self):
        import pyaudio
        pa = pyaudio.PyAudio()

        out_index = resolve_audio_device(pa, self.out_device, is_input=False, purpose="スピーカー")
        if out_index is None:
            pa.terminate()
            raise RuntimeError("[スピーカー] 出力デバイスが見つかりません")
        info = pa.get_device_info_by_index(out_index)
        print(f"[スピーカー] {info['name']} device={out_index}, 48000Hz")

        stream = pa.open(
            format=pyaudio.paInt16,
            channels=1,
            rate=48000,
            output=True,
            output_device_index=out_index,
            frames_per_buffer=4096,
        )
        stream.start_stream()
        try:
            while True:
                if self._abuf:
                    chunk = bytes(self._abuf[:4096])
                    self._abuf = self._abuf[4096:]
                    pcm = np.frombuffer(chunk, dtype=np.int16)
                    up = np.repeat(pcm, 2)  # 24000 → 48000
                    stream.write(up.tobytes())
                    self._speaking_until = time.monotonic() + ECHO_GATE_COOLDOWN_S  # Phase 5.1
                else:
                    await asyncio.sleep(0.01)
        finally:
            stream.stop_stream()
            stream.close()
            pa.terminate()

    async def recv_loop(self):
        async for raw in self._ws:
            try:
                ev = json.loads(raw)
            except Exception:
                continue
            t = ev.get("type", "")

            if t == "session.created":
                model = ev.get("session", {}).get("model", "unknown")
                print(f"[Realtime] セッション確立  model={model}")

            elif t == "session.updated":
                pass  # 確認済み

            elif t == "response.audio.delta":
                b64 = ev.get("delta", "")
                if b64:
                    self._abuf.extend(base64.b64decode(b64))

            elif t == "response.audio_transcript.delta":
                print(ev.get("delta", ""), end="", flush=True)

            elif t == "response.text.delta":
                print(ev.get("delta", ""), end="", flush=True)

            elif t == "response.function_call_arguments.done":
                fname = ev.get("name", "")
                if fname == "select_motion":
                    try:
                        name = json.loads(ev.get("arguments", "{}")).get("motion_name", "")
                        if name in self.motions:
                            m = self.motions[name]
                            print(f"\n[動作] → {name}  ({m['desc']}, {m['dur']:.1f}s)")
                            def _play(motion=m):
                                self.walker.switch_to_streaming()
                                self.player.play_once(motion, force=True)
                                while self.player.is_playing(): time.sleep(0.05)
                                self.walker.switch_to_planner()
                            threading.Thread(target=_play, daemon=True).start()
                        else:
                            print(f"\n[動作] 不明: {name}")
                    except Exception as e:
                        print(f"\n[動作エラー] {e}")
                elif fname == "walk_command":
                    try:
                        args = json.loads(ev.get("arguments", "{}"))
                        direction = args.get("direction", "stop")
                        steps = max(1, min(10, int(args.get("steps", 1) or 1)))
                        print(f"\n[歩行] → {direction} x{steps}")
                        def _walk(d=direction, n=steps):
                            if d == "forward":
                                self.walker.run_action(lambda: self.walker.walk_forward(n))
                            elif d == "backward":
                                self.walker.run_action(lambda: self.walker.walk_backward(n))
                            elif d == "turn_left":
                                self.walker.run_action(lambda: self.walker.turn_left(n))
                            elif d == "turn_right":
                                self.walker.run_action(lambda: self.walker.turn_right(n))
                            else:
                                self.walker.stop()
                        _walk()
                    except Exception as e:
                        print(f"\n[歩行エラー] {e}")
                elif fname == "navigate_to":
                    try:
                        args     = json.loads(ev.get("arguments", "{}"))
                        location = args.get("location", "entrance")
                        facing   = args.get("facing")
                        cell     = _NAMED_LOCATIONS.get(location)
                        if cell is None:
                            print(f"\n[Nav] 不明な場所: {location}")
                        elif self.navigator is None:
                            print("\n[Nav] ナビゲーター未初期化")
                        else:
                            xi, yi = cell
                            facing_deg = _FACING_ANGLES.get(facing) if facing else None
                            print(f"\n[Nav] → {location} ({xi},{yi})"
                                  + (f"  向き={facing}" if facing else ""))
                            def _nav(xi=xi, yi=yi, fd=facing_deg):
                                self.player.stop()
                                self.walker.switch_to_planner()
                                with self._nav_lock:
                                    # Phase 4.2: watchdog — NAV_TIMEOUT_S 超過で強制停止
                                    done = threading.Event()
                                    result: list = [False]
                                    def _run():
                                        result[0] = self.navigator.goto(xi, yi, facing_deg=fd)
                                        done.set()
                                    t = threading.Thread(target=_run, daemon=True)
                                    t.start()
                                    if not done.wait(timeout=NAV_TIMEOUT_S):
                                        print(f"\n[Nav警告] タイムアウト({NAV_TIMEOUT_S}s) — 強制停止")
                                        self.walker.stop()
                            threading.Thread(target=_nav, daemon=True).start()
                    except Exception as e:
                        print(f"\n[Navエラー] {e}")

                elif fname == "point_at":
                    try:
                        args     = json.loads(ev.get("arguments", "{}"))
                        location = args.get("location", "")
                        cell     = _NAMED_LOCATIONS.get(location)
                        if cell is None or self.navigator is None:
                            motion_name = "point_forward"
                        else:
                            tx, ty = cell
                            rx, ry = self.navigator.cell
                            motion_name = _pointing_motion(
                                rx, ry, self.walker._facing_angle, tx, ty)
                        print(f"\n[指さし] → {location}  使用モーション: {motion_name}")
                        m = self.motions.get(motion_name) or self.motions.get("point_forward")
                        if m:
                            def _point(motion=m):
                                self.walker.switch_to_streaming()
                                self.player.play_once(motion, force=True)
                                while self.player.is_playing():
                                    time.sleep(0.05)
                                self.walker.switch_to_planner()
                            threading.Thread(target=_point, daemon=True).start()
                    except Exception as e:
                        print(f"\n[指さしエラー] {e}")

                elif fname == "look_around":
                    print("\n[Vision] シーン解析中...")
                    # 実際の結果送信は output_item.done で行う

            elif t == "response.output_item.done":
                item = ev.get("item", {})
                if item.get("type") == "function_call":
                    fname    = item.get("name", "")
                    call_id  = item.get("call_id", "")
                    if fname == "look_around":
                        asyncio.create_task(self._submit_scene_analysis(call_id))
                    else:
                        asyncio.create_task(self._submit_function_result(call_id))

            elif t == "response.audio.done":
                print()

            elif t == "input_audio_buffer.speech_started":
                if time.monotonic() < self._speaking_until:
                    # Phase 5.1: エコー由来の誤検出 — ゲートを延長して無視
                    self._speaking_until = time.monotonic() + ECHO_GATE_COOLDOWN_S
                    print("\n[Echo] TTS中に音声検出（エコーとして無視）")
                else:
                    print("\n🎤 [話し中...]")
                    self._abuf = bytearray()    # ロボット音声を即時停止（full-duplex）
                    self.walker.stop()          # 移動・ナビゲーション中断
                    self._schedule_backchannel()  # 1.5s 後に相づち nod

            elif t == "input_audio_buffer.speech_stopped":
                self._cancel_backchannel()
                print("✅ [認識中...]")

            elif t == "conversation.item.input_audio_transcription.completed":
                tr = ev.get("transcript", "")
                if tr:
                    print(f"\n👤 ユーザー: {tr}")

            elif t == "response.created":
                print("🤖 G1: ", end="", flush=True)


            elif t == "error":
                print(f"\n[エラー] {ev.get('error', ev)}")

    async def _submit_function_result(self, call_id: str):
        """Function call の結果を送信して音声レスポンスを要求"""
        await self._send({
            "type": "conversation.item.create",
            "item": {
                "type": "function_call_output",
                "call_id": call_id,
                "output": "OK",
            }
        })
        await self._send({
            "type": "response.create",
            "response": {
                "modalities": ["text", "audio"],
                "tools": [],          # function call なしで音声のみ返す
                "tool_choice": "none",
            }
        })

    async def ptt_loop(self):
        import pyaudio, keyboard
        pa = pyaudio.PyAudio()

        dev_index = resolve_audio_device(pa, self.mic_device, is_input=True, purpose="マイク")
        if dev_index is None:
            pa.terminate()
            raise RuntimeError("[マイク] 入力デバイスが見つかりません")

        print("スペースキーを長押しで話してください。Ctrl+C で終了。\n")
        while True:
            print(">>> スペースキーを長押し...", end="", flush=True)
            await asyncio.get_event_loop().run_in_executor(None, keyboard.wait, "space")
            print(" [録音中]", end="", flush=True)
            recorded = []

            def cb(in_data, frame_count, time_info, status):
                recorded.append(np.frombuffer(in_data, dtype=np.int16).copy())
                return (None, pyaudio.paContinue)

            stream = pa.open(
                format=pyaudio.paInt16,
                channels=1,
                rate=MIC_RATE,
                input=True,
                input_device_index=dev_index,
                frames_per_buffer=2048,
                stream_callback=cb,
            )
            stream.start_stream()
            await asyncio.get_event_loop().run_in_executor(None, keyboard.wait, "space", True)
            stream.stop_stream()
            stream.close()
            print(" [送信中]")
            if recorded:
                pcm = np.concatenate(recorded)[::2]
                i16 = pcm.clip(-32768, 32767).astype(np.int16)
                b64 = base64.b64encode(i16.tobytes()).decode()
                await self._send({"type": "input_audio_buffer.append", "audio": b64})
                await self._send({"type": "input_audio_buffer.commit"})
                await self._send({"type": "response.create"})

    async def run(self):
        await self.connect()
        if self.vad:
            await asyncio.gather(
                self.stream_mic(), self.recv_loop(), self.play_audio()
            )
        else:
            await asyncio.gather(
                self.ptt_loop(), self.recv_loop(), self.play_audio()
            )


# ── エントリポイント ──────────────────────────────────────────

def main():
    global MIC_DEVICE_ID, OUT_DEVICE_ID
    if _restart_pipewire_services_if_available():
        time.sleep(2)
    p = argparse.ArgumentParser(description="G1 Realtime 音声対話")
    p.add_argument("--ptt",        action="store_true")
    p.add_argument("--mic-device", default=MIC_DEVICE_ID,
                   help="入力デバイス番号、またはデバイス名の一部")
    p.add_argument("--out-device", default=OUT_DEVICE_ID,
                   help="出力デバイス番号、またはデバイス名の一部")
    p.add_argument("--list-audio-devices", action="store_true",
                   help="利用可能な音声デバイス一覧を表示して終了")
    p.add_argument("--motion-dir", default=G1_MOTION_DIR,
                   help=f"動作ライブラリルート (デフォルト: {G1_MOTION_DIR})")
    p.add_argument("--zmq-port",    type=int,   default=ZMQ_PORT)
    p.add_argument("--camera-port", type=int,   default=5555,
                   help="カメラ映像 ZMQ ポート (デフォルト: 5555)")
    p.add_argument("--no-camera",   action="store_true",
                   help="カメラキャプチャを無効化")
    args = p.parse_args()

    if args.list_audio_devices:
        print_audio_devices()
        return

    MIC_DEVICE_ID = _normalize_device_selector(args.mic_device)
    OUT_DEVICE_ID = _normalize_device_selector(args.out_device)

    ctx  = zmq.Context()
    sock = ctx.socket(zmq.PUB)
    sock.bind(f"tcp://*:{args.zmq_port}")
    print(f"[ZMQ] tcp://*:{args.zmq_port}")
    time.sleep(0.5)

    motions = load_motions(args.motion_dir)

    # グリッドナビゲーション初期化
    grid = build_grid()
    start_cell = _nearest_grid(grid, 0.0, 0.0)
    start_xi, start_yi = start_cell if start_cell else (0, 0)
    print(f"[Nav] グリッド: {len(grid)} セル  起動位置 ({start_xi},{start_yi})")

    player = MotionPlayer(sock)
    walker = WalkerController(sock)
    walker.start_planner()
    navigator = GridNavigator(walker, grid, start_xi, start_yi)

    # カメラ購読（Phase 3.1 → CameraFrameSubscriber）
    camera = None
    camera_enabled = VISION_ENABLED or (not args.no_camera)
    if camera_enabled:
        if VISION_ENABLED and OPENAI_REALTIME_MODEL not in VISION_SUPPORTED_MODELS:
            print(f"[Vision] 警告: {OPENAI_REALTIME_MODEL} は画像入力対応確認済みモデルではありません。")
        camera = CameraFrameSubscriber(
            zmq_url=VISION_CAMERA_ZMQ if VISION_ENABLED else f"tcp://localhost:{args.camera_port}",
            camera=VISION_CAMERA if VISION_ENABLED else "ego_view",
            max_staleness_sec=VISION_MAX_STALENESS_SEC,
            detail=VISION_DETAIL,
        )
        camera.start()

    kb = KeyboardController(walker, player)
    kb.start()
    print(f"✅ 起動完了  モード: {'PTT' if args.ptt else 'VAD'}  動作数: {len(motions)}")
    print("⚠️  手動制御: W=前進 S=後退 A=左旋回 D=右旋回 Space=停止\n")

    try:
        asyncio.run(
            RealtimeDialogue(
                motions,
                player,
                walker,
                navigator=navigator,
                camera=camera,
                vad=not args.ptt,
                mic_device=MIC_DEVICE_ID,
                out_device=OUT_DEVICE_ID,
            ).run()
        )
    except KeyboardInterrupt:
        print("\n終了します")
    finally:
        player.stop()
        kb.stop()
        if camera:
            camera.stop()
        sock.close()
        ctx.term()


if __name__ == "__main__":
    main()
