#!/usr/bin/env python3
"""MuJoCo カメラ映像ビューア — ZMQ ストリームをウィンドウ表示する。

Usage:
    python src/dialogue_system/mujoco_viewer.py
    python src/dialogue_system/mujoco_viewer.py --camera ego_view
    python src/dialogue_system/mujoco_viewer.py --zmq tcp://localhost:5555

終了: ウィンドウを閉じるか q キーを押す。
"""

from __future__ import annotations

import argparse
import base64
import sys
import threading
import time

import msgpack
import numpy as np
import zmq


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="MuJoCo カメラ映像ビューア")
    p.add_argument("--zmq", default="tcp://localhost:5555", help="ZMQ subscribe URL")
    p.add_argument("--camera", default="ego_view", help="表示するカメラ名")
    p.add_argument("--fps", type=float, default=30.0, help="表示フレームレート上限")
    return p.parse_args()


def main() -> None:
    try:
        import cv2
    except ImportError:
        print("[viewer] cv2 が見つかりません: pip install opencv-python")
        sys.exit(1)

    args = parse_args()

    latest: dict = {"frame": None, "camera": None}
    stop = threading.Event()

    def receiver() -> None:
        ctx = zmq.Context()
        sock = ctx.socket(zmq.SUB)
        sock.setsockopt(zmq.SUBSCRIBE, b"")
        sock.setsockopt(zmq.CONFLATE, 1)
        sock.setsockopt(zmq.RCVTIMEO, 200)
        sock.connect(args.zmq)
        print(f"[viewer] ZMQ 接続: {args.zmq}")
        try:
            while not stop.is_set():
                try:
                    raw = sock.recv()
                except zmq.Again:
                    continue
                payload = msgpack.unpackb(raw, raw=False)
                images = payload.get("images", {}) if isinstance(payload, dict) else {}
                if not images:
                    continue

                cam = args.camera if args.camera in images else next(iter(images))
                encoded = images.get(cam)
                if not encoded:
                    continue

                jpg = base64.b64decode(encoded)
                arr = np.frombuffer(jpg, dtype=np.uint8)
                img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                if img is not None:
                    latest["frame"] = img
                    latest["camera"] = cam
        finally:
            sock.close(0)
            ctx.term()

    t = threading.Thread(target=receiver, daemon=True)
    t.start()

    interval = 1.0 / args.fps
    print("[viewer] 起動中… ウィンドウが開くまで少し待ってください (q で終了)")

    win_name = f"MuJoCo: {args.camera}"
    cv2.namedWindow(win_name, cv2.WINDOW_NORMAL)

    delay_ms = max(1, int(1000 / args.fps))

    while True:
        frame = latest["frame"]

        if frame is not None:
            cv2.imshow(win_name, frame)
        else:
            blank = np.zeros((240, 320, 3), dtype=np.uint8)
            cv2.putText(blank, "Waiting for sim...", (20, 120),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 200), 1)
            cv2.imshow(win_name, blank)

        # waitKey に十分な時間を渡して Qt イベントループを回す
        key = cv2.waitKey(delay_ms) & 0xFF
        if key in (ord("q"), 27):  # q または ESC
            break
        try:
            if cv2.getWindowProperty(win_name, cv2.WND_PROP_AUTOSIZE) < 0:
                break  # ウィンドウが閉じられた
        except cv2.error:
            break

    stop.set()
    cv2.destroyAllWindows()
    print("[viewer] 終了")


if __name__ == "__main__":
    main()
