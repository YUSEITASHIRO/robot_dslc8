#!/usr/bin/env python3
"""
g1_simple_walk.py — キーボード (WASD) で G1 を歩かせる

起動手順:
  Terminal 1: source .venv_sim/bin/activate
              python gear_sonic/scripts/run_sim_loop.py
  Terminal 2: bash deploy.sh --input-type zmq --zmq-port 5556 sim
              → Y キーで起動
  Terminal 3: python src/move/g1_simple_walk.py

操作:
  W   前進
  S   後退
  A   左旋回
  D   右旋回
  Q   左横移動
  E   右横移動
  スペース  停止
  X / Ctrl-C  終了
"""

import json
import math
import struct
import sys
import termios
import time
import tty

import numpy as np
import zmq

# ─── 設定 ────────────────────────────────────────
PORT        = 5556
HEADER_SIZE = 1280
TURN_STEP   = math.radians(15)

# ─── ZMQ ─────────────────────────────────────────
ctx  = zmq.Context()
sock = ctx.socket(zmq.PUB)
sock.bind(f"tcp://*:{PORT}")
print(f"[ZMQ] tcp://*:{PORT}")
time.sleep(0.5)

# ─── メッセージ送信 ───────────────────────────────
def send_msg(topic, fields, data):
    header = {"v": 1, "endian": "le", "count": 1, "fields": fields}
    hj = json.dumps(header).encode()
    hb = hj + b"\x00" * (HEADER_SIZE - len(hj))
    sock.send(topic + hb + data)

def send_command(start=True, stop=False, planner=True):
    fields = [
        {"name": "start",   "dtype": "u8", "shape": [1]},
        {"name": "stop",    "dtype": "u8", "shape": [1]},
        {"name": "planner", "dtype": "u8", "shape": [1]},
    ]
    send_msg(b"command", fields, struct.pack("BBB", int(start), int(stop), int(planner)))

def send_planner(mode, movement, facing, speed=-1.0):
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
    send_msg(b"planner", fields, data)

# ─── キーボード ───────────────────────────────────
def get_key():
    fd  = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1).lower()
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)

def fv(angle):
    return [math.cos(angle), math.sin(angle), 0.0]

# ─── 起動 ─────────────────────────────────────────
facing = 0.0

print("[起動] planner モード開始...")
send_command(start=True, stop=False, planner=True)
time.sleep(1.5)
send_planner(0, [0, 0, 0], fv(facing))
print("[起動] 完了 — IDLE 待機中")

print("""
操作キー:
  W = 前進      S = 後退
  A = 左旋回    D = 右旋回
  Q = 左横移動  E = 右横移動
  スペース = 停止
  X = 終了
""")

try:
    while True:
        key = get_key()

        if key in ("x", "\x03"):
            break

        elif key == "w":
            mv = fv(facing)
            print(f"[W] 前進  facing={math.degrees(facing):.0f}°")
            send_planner(2, mv, mv)
            time.sleep(0.3)
            send_planner(0, [0, 0, 0], fv(facing))

        elif key == "s":
            mv = [-math.cos(facing), -math.sin(facing), 0.0]
            print(f"[S] 後退  facing={math.degrees(facing):.0f}°")
            send_planner(2, mv, fv(facing))
            time.sleep(0.3)
            send_planner(0, [0, 0, 0], fv(facing))

        elif key == "a":
            facing += TURN_STEP
            print(f"[A] 左旋回 → facing={math.degrees(facing):.0f}°")
            for _ in range(5):
                send_planner(2, [0, 0, 0], fv(facing))
                time.sleep(0.1)

        elif key == "d":
            facing -= TURN_STEP
            print(f"[D] 右旋回 → facing={math.degrees(facing):.0f}°")
            for _ in range(5):
                send_planner(2, [0, 0, 0], fv(facing))
                time.sleep(0.1)

        elif key == "q":
            left = [math.cos(facing + math.pi / 2),
                    math.sin(facing + math.pi / 2), 0.0]
            print("[Q] 左横移動")
            for _ in range(5):
                send_planner(2, left, fv(facing))
                time.sleep(0.1)

        elif key == "e":
            right = [math.cos(facing - math.pi / 2),
                     math.sin(facing - math.pi / 2), 0.0]
            print("[E] 右横移動")
            for _ in range(5):
                send_planner(2, right, fv(facing))
                time.sleep(0.1)

        elif key == " ":
            print("[スペース] 停止")
            for _ in range(3):
                send_planner(0, [0, 0, 0], fv(facing))
                time.sleep(0.1)

except KeyboardInterrupt:
    pass
finally:
    print("\n[終了] IDLE に戻す...")
    send_planner(0, [0, 0, 0], fv(facing))
    time.sleep(0.3)
    send_command(start=False, stop=False, planner=True)
    sock.close()
    ctx.term()
    print("終了")
