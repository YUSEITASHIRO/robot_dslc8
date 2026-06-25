#!/usr/bin/env python3
"""
g1_realtime_motion_test.py — インタラクティブ モーション生成 → 即時再生

【起動前の準備】
  1. Docker コンテナで text encoder server を起動:
       cd extern/kimodo
       docker compose up text-encoder
     → Gradio server が port 9550 で起動
     → GPU メモリ ~10GB 使用 (LLM2Vec llama-8B)

  2. このスクリプトを起動:
       python src/motion/g1_realtime_motion_test.py
     → TEXT_ENCODER_URL=http://127.0.0.1:9550/ (デフォルト) で自動接続
     → diffusion model が GPU に乗る (~8GB)

【使い方】
  プロンプトを日本語・英語で入力して Enter → 生成 → 即時再生
  コマンド一覧は h または ? で表示

【コマンド例】
  wave both hands in greeting
  dur 5          # 生成秒数変更
  steps 50       # Diffusion ステップ数変更 (速度優先)
  play           # 最後の動作を再再生
  stop           # 再生停止
"""

import json
import math
import os
import struct
import sys
import threading
import time
from typing import Optional

import numpy as np
import zmq
from scipy.interpolate import interp1d
from scipy.spatial.transform import Rotation, Slerp

# ── パス設定 ──────────────────────────────────────────────────────
sys.path.insert(0, "/home/unitree-g1/Documents/G1/kimodo")

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# ── 設定 ──────────────────────────────────────────────────────────
ZMQ_PORT    = 5556
HEADER_SIZE = 1280
SONIC_FPS   = 50
CHUNK_SIZE  = 5

MODEL_NAME        = "kimodo-g1-rp"
KIMODO_FPS        = 30
DEFAULT_DUR       = 10.0        # 生成秒数デフォルト
DEFAULT_STEPS     = 50         # Diffusion ステップ数 (100=高品質, 50=速度優先)
BLEND_FRAMES      = 12         # 境界ブレンドフレーム数 (50fps)
TEXT_ENCODER_URL  = os.environ.get("TEXT_ENCODER_URL", "http://127.0.0.1:9550/")

# Kimodo(MuJoCo順) → IsaacLab順 (SONIC ZMQ 期待形式)
MUJOCO_TO_ISAACLAB = np.array([
     0,  6, 12,  1,  7, 13,  2,  8, 14,
     3,  9, 15, 22,  4, 10, 16, 23,
     5, 11, 17, 24, 18, 25, 19, 26,
    20, 27, 21, 28
], dtype=np.int32)

STANDING_JP = np.zeros(29, dtype=np.float32)
STANDING_BQ = np.array([1., 0., 0., 0.], dtype=np.float32)

STAND_PROMPT = (
    "A humanoid robot standing completely still in a natural upright position, "
    "arms relaxed at sides, feet together."
)

# 腕関節インデックス (IsaacLab順): blend 対象外
ARM_IL = np.array([11, 12, 15, 16, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28], dtype=np.int32)


# ── モーション生成ユーティリティ ──────────────────────────────────

def resample(arr: np.ndarray, src_fps: float, dst_fps: float) -> np.ndarray:
    T = arr.shape[0]
    T_new = max(2, int(round(T * dst_fps / src_fps)))
    t_src = np.linspace(0.0, 1.0, T)
    t_dst = np.linspace(0.0, 1.0, T_new)
    return interp1d(t_src, arr, axis=0, kind="linear")(t_dst).astype(arr.dtype)


def slerp_quat(q_start: np.ndarray, q_end: np.ndarray, n: int) -> np.ndarray:
    r_start = Rotation.from_quat(q_start[[1, 2, 3, 0]])
    r_end   = Rotation.from_quat(q_end[[1, 2, 3, 0]])
    times   = np.linspace(0.0, 1.0, n)
    rots    = Slerp([0.0, 1.0], Rotation.concatenate([r_start, r_end]))(times)
    xyzw    = rots.as_quat()
    return np.column_stack([xyzw[:, 3], xyzw[:, :3]]).astype(np.float32)


def blend_boundaries(jp: np.ndarray, bq: np.ndarray, n: int) -> tuple:
    jp = jp.copy(); bq = bq.copy()
    alpha = np.linspace(0.0, 1.0, n, dtype=np.float32)[:, None]
    jp[:n] = (1 - alpha) * STANDING_JP + alpha * jp[:n]
    bq[:n] = slerp_quat(STANDING_BQ, bq[n - 1], n)
    alpha = np.linspace(0.0, 1.0, n, dtype=np.float32)[:, None]
    jp[-n:] = (1 - alpha) * jp[-n] + alpha * STANDING_JP
    bq[-n:] = slerp_quat(bq[-(n + 1)], STANDING_BQ, n)
    return jp, bq


def compute_jv(jp: np.ndarray) -> np.ndarray:
    jv = np.zeros_like(jp)
    if len(jp) > 1:
        jv[:-1] = (jp[1:] - jp[:-1]) * SONIC_FPS
        jv[-1]  = jv[-2]
    return jv


def generate_motion(model, converter, device: str,
                    prompt: str, dur: float, steps: int) -> dict:
    """プロンプトからモーションを生成し dict(jp, jv, bq, dur_sec) を返す。"""
    num_frames = int(round(dur * KIMODO_FPS))

    print(f"[Gen] 生成中… dur={dur:.1f}s  steps={steps}  "
          f"({num_frames}f@{KIMODO_FPS}fps)", flush=True)
    t0 = time.perf_counter()

    import torch
    output = model(
        prompts=[prompt],
        num_frames=[num_frames],
        num_denoising_steps=steps,
        multi_prompt=False,
        num_samples=1,
    )
    qpos    = converter.dict_to_qpos(output, device)
    qpos_np = qpos.cpu().numpy() if hasattr(qpos, "cpu") else np.array(qpos)
    q       = qpos_np[0]                               # (T_30fps, 36)

    jp_30 = q[:, 7:].astype(np.float32)[:, MUJOCO_TO_ISAACLAB]
    bq_30 = q[:, 3:7].astype(np.float32)              # wxyz

    jp_50 = resample(jp_30, KIMODO_FPS, SONIC_FPS)
    bq_50 = resample(bq_30, KIMODO_FPS, SONIC_FPS)
    bq_50 /= np.linalg.norm(bq_50, axis=1, keepdims=True).clip(1e-8)

    blend_n      = min(BLEND_FRAMES, len(jp_50) // 4)
    arm_backup   = jp_50[:, ARM_IL].copy()
    jp_50, bq_50 = blend_boundaries(jp_50, bq_50, blend_n)
    jp_50[:, ARM_IL] = arm_backup

    elapsed = time.perf_counter() - t0
    print(f"[Gen] 完了  {len(jp_50)}フレーム ({len(jp_50)/SONIC_FPS:.1f}s @ {SONIC_FPS}fps)  "
          f"生成時間: {elapsed:.1f}s", flush=True)

    return {"jp": jp_50, "jv": compute_jv(jp_50), "bq": bq_50,
            "dur_sec": len(jp_50) / SONIC_FPS, "prompt": prompt}


# ── ZMQ ヘルパー ─────────────────────────────────────────────────

def send_pose(sock, joint_pos, joint_vel, body_quat, frame_index):
    N = len(joint_pos)
    header = {
        "v": 1, "endian": "le", "count": N,
        "fields": [
            {"name": "joint_pos",   "dtype": "f32", "shape": [N, 29]},
            {"name": "joint_vel",   "dtype": "f32", "shape": [N, 29]},
            {"name": "body_quat_w", "dtype": "f32", "shape": [N,  4]},
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


# ── MotionPlayer ─────────────────────────────────────────────────

class MotionPlayer:
    def __init__(self, sock):
        self._sock   = sock
        self._thread: Optional[threading.Thread] = None
        self._stop   = threading.Event()
        self._fi     = 0

    def is_playing(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def stop(self):
        self._stop.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.0)
        self._thread = None
        self._stop.clear()

    def play(self, motion: dict):
        """再生をブロッキングで実行 (完了まで待つ)。"""
        self.stop()
        jp = motion["jp"]; jv = motion["jv"]; bq = motion["bq"]
        N  = len(jp)
        self._stop.clear()

        def _run():
            for i in range(0, N, CHUNK_SIZE):
                if self._stop.is_set():
                    return
                n = min(CHUNK_SIZE, N - i)
                send_pose(self._sock, jp[i:i+n], jv[i:i+n], bq[i:i+n], self._fi)
                self._fi += n
                time.sleep(n / SONIC_FPS)

        self._thread = threading.Thread(target=_run, daemon=True)
        self._thread.start()
        self._thread.join()   # ブロッキング: 完了まで待つ


# ── WalkerController (SONIC モード切替用) ─────────────────────────

class WalkerController:
    def __init__(self, sock):
        self._sock                 = sock
        self._planner_mode         = False
        self._zmq_streaming_active = False
        self._facing_angle         = 0.0

    def _send_msg(self, topic, fields, data):
        hj = json.dumps({"v": 1, "endian": "le", "count": 1, "fields": fields}).encode()
        hb = hj + b"\x00" * (HEADER_SIZE - len(hj))
        self._sock.send(topic + hb + data)

    def send_command(self, start=True, stop=False, planner=True):
        fields = [{"name": "start",   "dtype": "u8", "shape": [1]},
                  {"name": "stop",    "dtype": "u8", "shape": [1]},
                  {"name": "planner", "dtype": "u8", "shape": [1]}]
        self._send_msg(b"command", fields, struct.pack("BBB", int(start), int(stop), int(planner)))

    def send_planner(self, mode, movement, facing, speed=-1.0):
        fields = [{"name": "mode",     "dtype": "i32", "shape": [1]},
                  {"name": "movement", "dtype": "f32", "shape": [3]},
                  {"name": "facing",   "dtype": "f32", "shape": [3]},
                  {"name": "speed",    "dtype": "f32", "shape": [1]},
                  {"name": "height",   "dtype": "f32", "shape": [1]}]
        data  = struct.pack("<i", mode)
        data += struct.pack("<fff", *movement)
        data += struct.pack("<fff", *facing)
        data += struct.pack("<ff",  speed, -1.0)
        self._send_msg(b"planner", fields, data)

    def _fv(self):
        a = self._facing_angle
        return [math.cos(a), math.sin(a), 0.0]

    def start_planner(self):
        if not self._planner_mode:
            self.send_command(start=True, stop=False, planner=True)
            time.sleep(1.0)
            self.send_planner(0, [0, 0, 0], self._fv())
            self._planner_mode = True
            print("[Walker] planner モード開始 (アイドル待機)")

    def switch_to_streaming(self):
        # 2回目以降はトグル問題回避のため P→S→P→S の二重送信
        if self._zmq_streaming_active:
            self.send_command(start=True, stop=False, planner=False)
            time.sleep(0.1)
            self.send_command(start=True, stop=False, planner=True)
            time.sleep(0.1)
        self.send_command(start=True, stop=False, planner=False)
        self._planner_mode = False
        self._zmq_streaming_active = True
        time.sleep(0.2)

    def switch_to_planner(self):
        self.send_command(start=True, stop=False, planner=True)
        self._planner_mode = True
        time.sleep(0.5)
        self.send_planner(0, [0, 0, 0], self._fv())

    def stop(self):
        if not self._planner_mode:
            self.send_command(start=True, stop=False, planner=True)
            time.sleep(0.3)
            self._planner_mode = True
        self.send_planner(0, [0, 0, 0], self._fv())


# ── Interactive CLI ───────────────────────────────────────────────

_HELP = """\
─────────────────────────────────────────────────────
【動作生成・再生】
  <プロンプト>          テキストから動作を生成して即時再生
                         例: wave both hands in greeting
                         例: bow forward at 45 degrees
  play                  最後に生成した動作を再再生

【パラメータ設定】
  dur <秒>              生成秒数を変更 (デフォルト: 3.0)
                         例: dur 5
  steps <n>             Diffusion ステップ数 (デフォルト: 50)
                         50=速い, 100=高品質
                         例: steps 100

【制御】
  start                 SONIC planner モード開始
  stop                  再生停止
  status / s            現在の設定を表示

【その他】
  q / quit              終了
  h / ?                 このヘルプを表示
─────────────────────────────────────────────────────"""


def run_cli(walker: WalkerController, player: MotionPlayer,
            model, converter, device: str):
    dur    = DEFAULT_DUR
    steps  = DEFAULT_STEPS
    last_motion: Optional[dict] = None

    print(_HELP)
    print(f"\n設定: dur={dur:.1f}s  steps={steps}  "
          f"text_encoder={TEXT_ENCODER_URL}")
    print("プロンプトを入力して動作を生成・再生します\n")

    while True:
        try:
            raw = input("prompt> ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            break

        if not raw:
            continue

        parts = raw.split()
        cmd   = parts[0].lower()

        if cmd in ("q", "quit"):
            break

        elif cmd in ("h", "?", "help"):
            print(_HELP)

        elif cmd in ("status", "s"):
            print(f"  dur={dur:.1f}s  steps={steps}")
            print(f"  text_encoder_url={TEXT_ENCODER_URL}")
            if last_motion:
                print(f"  最後の動作: \"{last_motion['prompt'][:60]}\"  "
                      f"{last_motion['dur_sec']:.1f}s")
            else:
                print("  最後の動作: なし")

        elif cmd == "dur":
            if len(parts) < 2:
                print(f"  現在の dur={dur:.1f}s  使い方: dur <秒>  例: dur 5")
            else:
                try:
                    v = float(parts[1])
                    if v < 1.0:
                        raise ValueError
                    dur = v
                    print(f"  dur → {dur:.1f}s")
                except ValueError:
                    print("  1.0 以上の数値を指定してください")

        elif cmd == "steps":
            if len(parts) < 2:
                print(f"  現在の steps={steps}  使い方: steps <n>  例: steps 100")
            else:
                try:
                    v = int(parts[1])
                    if v < 1:
                        raise ValueError
                    steps = v
                    print(f"  steps → {steps}")
                except ValueError:
                    print("  1 以上の整数を指定してください")

        elif cmd == "start":
            walker.start_planner()

        elif cmd == "stop":
            player.stop()
            walker.stop()

        elif cmd == "play":
            if last_motion is None:
                print("  まだ動作が生成されていません。プロンプトを入力してください。")
            else:
                print(f"[Play] 再再生: \"{last_motion['prompt'][:60]}\"")
                walker.switch_to_streaming()
                player.play(last_motion)
                walker.switch_to_planner()
                print("[Play] 完了")

        else:
            # プロンプトとして扱う
            prompt = raw
            try:
                motion = generate_motion(model, converter, device, prompt, dur, steps)
                last_motion = motion
                print(f"[Play] 再生開始 ({motion['dur_sec']:.1f}s)…", flush=True)
                walker.switch_to_streaming()
                player.play(motion)
                walker.switch_to_planner()
                print("[Play] 完了")
            except Exception as e:
                print(f"[ERROR] 生成/再生エラー: {e}")
                import traceback
                traceback.print_exc()
                # エラー後は planner に戻す
                try:
                    walker.switch_to_planner()
                except Exception:
                    pass


# ── Entry point ──────────────────────────────────────────────────

def main():
    import argparse
    ap = argparse.ArgumentParser(
        description="インタラクティブ Kimodo モーション生成 → 即時再生",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    ap.add_argument("--zmq-port", type=int, default=ZMQ_PORT,
                    help=f"SONIC ZMQ ポート (デフォルト: {ZMQ_PORT})")
    ap.add_argument("--dur",   type=float, default=DEFAULT_DUR,
                    help=f"生成秒数デフォルト (デフォルト: {DEFAULT_DUR})")
    ap.add_argument("--steps", type=int,   default=DEFAULT_STEPS,
                    help=f"Diffusion ステップ数 (デフォルト: {DEFAULT_STEPS})")
    ap.add_argument("--text-encoder-url", default=TEXT_ENCODER_URL,
                    help=f"Text encoder Gradio URL (デフォルト: {TEXT_ENCODER_URL})")
    args = ap.parse_args()

    # TEXT_ENCODER_URL を環境変数経由で渡す (kimodo.load_model が参照)
    os.environ["TEXT_ENCODER_URL"] = args.text_encoder_url

    # ── Kimodo モデルロード ──────────────────────────────────────
    print(f"[Setup] Text encoder: {args.text_encoder_url}")
    print(f"[Setup] Kimodo モデル読み込み中: {MODEL_NAME} …")
    import torch
    from kimodo import load_model
    from kimodo.exports.mujoco import MujocoQposConverter

    device = "cuda:0" if torch.cuda.is_available() else "cpu"
    print(f"[Setup] デバイス: {device}")

    model, resolved = load_model(MODEL_NAME, device=device, return_resolved_name=True)
    print(f"[Setup] ロード完了: {resolved}  fps={model.fps}")
    converter = MujocoQposConverter(model.skeleton)

    # ── ZMQ + SONIC 起動 ─────────────────────────────────────────
    ctx  = zmq.Context()
    sock = ctx.socket(zmq.PUB)
    sock.bind(f"tcp://*:{args.zmq_port}")
    time.sleep(0.3)

    walker = WalkerController(sock)
    player = MotionPlayer(sock)

    print(f"[Setup] ZMQ PUB :{args.zmq_port}  dur={args.dur:.1f}s  steps={args.steps}")
    walker.start_planner()

    try:
        run_cli(walker, player, model, converter, device)
    finally:
        player.stop()
        walker.stop()
        sock.close()
        ctx.term()
        print("[終了]")


if __name__ == "__main__":
    main()
