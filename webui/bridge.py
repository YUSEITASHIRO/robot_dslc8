#!/usr/bin/env python3
"""
webui/bridge.py — G1 対話システム Web UI ブリッジサーバー

ZMQ カメラストリームを WebSocket でブラウザに中継し、
ブラウザからの ↑↓←→ キーを MuJoCo viewer (GLFW) に転送する。
内部プログラムへの変更は一切なし。

使い方:
  # ログファイルを監視する場合（プロセスは別途手動起動）
  python webui/bridge.py --sim-log /tmp/sim.log --dialogue-log /tmp/dial.log

  # サブプロセスとして起動して stdout を取り込む場合
  python webui/bridge.py \\
    --launch-sim     "python patches/.../run_sim.py" \\
    --launch-dialogue "python src/dialogue_system/g1_realtime_dialogue.py"

環境変数:
  CAMERA_ZMQ_PORT  ZMQ カメラポート（デフォルト: 5555）
  WEBUI_PORT       HTTP/WS ポート（デフォルト: 8765）
"""

import argparse
import asyncio
import ctypes
import json
import logging
import os
import re
import sys
import time
from pathlib import Path

try:
    import aiohttp
    from aiohttp import web
except ImportError:
    print("pip install aiohttp")
    sys.exit(1)

try:
    import msgpack
except ImportError:
    print("pip install msgpack")
    sys.exit(1)

try:
    import zmq
    import zmq.asyncio
except ImportError:
    print("pip install pyzmq")
    sys.exit(1)

log = logging.getLogger("bridge")

# ─── 接続中 WebSocket クライアント ───────────────────────────────────────────
_clients: set = set()


async def _broadcast(msg: dict):
    if not _clients:
        return
    text = json.dumps(msg, ensure_ascii=False)
    dead = set()
    for ws in list(_clients):
        try:
            await ws.send_str(text)
        except Exception:
            dead.add(ws)
    _clients -= dead


# ─── ZMQ カメラストリーム → WebSocket ────────────────────────────────────────
async def camera_loop(zmq_port: int):
    ctx = zmq.asyncio.Context()
    sock = ctx.socket(zmq.SUB)
    sock.setsockopt(zmq.SUBSCRIBE, b"")
    sock.setsockopt(zmq.CONFLATE, 1)
    sock.connect(f"tcp://localhost:{zmq_port}")
    log.info(f"Camera ZMQ listening on tcp://localhost:{zmq_port}")

    fps_t: dict = {}
    fps_v: dict = {}

    while True:
        try:
            raw = await asyncio.wait_for(sock.recv(), timeout=0.5)
            payload = msgpack.unpackb(raw, raw=False)
            images = payload.get("images", {})
            robot_vis = payload.get("robot_in_user_view")
            now = time.time()

            for cam, img_b64 in images.items():
                if not isinstance(img_b64, str):
                    continue
                dt = now - fps_t.get(cam, now) or 1e-3
                fps_t[cam] = now
                fps_v[cam] = 0.9 * fps_v.get(cam, 0.0) + 0.1 / dt

                msg: dict = {
                    "type": "camera",
                    "cam": cam,
                    "data": img_b64,
                    "fps": round(fps_v[cam], 1),
                }
                if cam == "user_eye" and robot_vis is not None:
                    msg["robot_visible"] = bool(robot_vis)
                await _broadcast(msg)

        except asyncio.TimeoutError:
            pass
        except Exception as e:
            log.debug(f"camera_loop: {e}")
            await asyncio.sleep(0.5)


# ─── ログ行パース & 配信 ─────────────────────────────────────────────────────
_TRANSCRIPT_RE = re.compile(r"\[(User|AI|Assistant)\][\s:]+(.+)", re.IGNORECASE)
_MOTION_RE     = re.compile(r"\[Motion\].*?play(?:ing)?[:：]?\s*(\S+)", re.IGNORECASE)
_PHASE_RE      = re.compile(r"\[Phase\]\s+→\s+(\S+)", re.IGNORECASE)


async def _dispatch_line(source: str, line: str):
    await _broadcast({"type": "log", "source": source, "text": line})

    m = _TRANSCRIPT_RE.search(line)
    if m:
        role = "user" if m.group(1).lower() == "user" else "assistant"
        await _broadcast({"type": "transcript", "role": role, "text": m.group(2).strip()})
        return

    m = _MOTION_RE.search(line)
    if m:
        await _broadcast({"type": "motion", "name": m.group(1), "ts": time.time()})

    m = _PHASE_RE.search(line)
    if m:
        await _broadcast({"type": "phase", "phase": m.group(1)})


# ─── サブプロセス起動 & stdout キャプチャ ───────────────────────────────────
async def _pipe_reader(stream, source: str):
    while True:
        line = await stream.readline()
        if not line:
            break
        await _dispatch_line(source, line.decode("utf-8", errors="replace").rstrip())


async def launch_subprocess(source: str, cmd: str):
    log.info(f"[{source}] launching: {cmd}")
    proc = await asyncio.create_subprocess_shell(
        cmd,
        stdout=asyncio.subprocess.PIPE,
        stderr=asyncio.subprocess.STDOUT,
    )
    await _broadcast({"type": "log", "source": source, "text": f"▶ launched: {cmd}"})
    await _pipe_reader(proc.stdout, source)
    rc = await proc.wait()
    await _broadcast({"type": "log", "source": source, "text": f"■ exited rc={rc}"})


# ─── ログファイル tail ────────────────────────────────────────────────────────
async def tail_log(source: str, path: str):
    p = Path(path)
    log.info(f"[{source}] watching: {p}")
    while not p.exists():
        await asyncio.sleep(1)
    with open(p, "r", encoding="utf-8", errors="replace") as f:
        f.seek(0, 2)
        while True:
            line = f.readline()
            if line:
                await _dispatch_line(source, line.rstrip())
            else:
                await asyncio.sleep(0.05)


# ─── MuJoCo GLFW ウィンドウへのキー注入 ─────────────────────────────────────
# GLFW キーコード (base_sim.py move_user と対応)
_WEB_TO_GLFW: dict[str, int] = {
    "ArrowUp":    265,  # 前進
    "ArrowDown":  264,  # 後退
    "ArrowLeft":  263,  # 左転向 (CCW)
    "ArrowRight": 262,  # 右転向 (CW)
    " ":           32,  # Space: 180°反転
}

# GLFW キーコード → Windows Virtual Key コード
_GLFW_TO_VK: dict[int, int] = {
    265: 0x26,  # VK_UP
    264: 0x28,  # VK_DOWN
    263: 0x25,  # VK_LEFT
    262: 0x27,  # VK_RIGHT
    32:  0x20,  # VK_SPACE
}

# GLFW キーコード → xdotool キー名 (Linux)
_GLFW_TO_XDO: dict[int, str] = {
    265: "Up",
    264: "Down",
    263: "Left",
    262: "Right",
    32:  "space",
}


def _find_mujoco_hwnd() -> int | None:
    """MuJoCo GLFW ウィンドウのハンドルを取得する (Windows のみ)。"""
    try:
        import win32gui
        found = []

        def _cb(hwnd, _):
            if win32gui.IsWindowVisible(hwnd):
                title = win32gui.GetWindowText(hwnd)
                if "mujoco" in title.lower():
                    found.append(hwnd)

        win32gui.EnumWindows(_cb, None)
        return found[0] if found else None
    except Exception:
        return None


def _inject_key_sync(key_str: str):
    """ブラウザからの key 文字列を MuJoCo viewer に転送する (blocking)。"""
    glfw_key = _WEB_TO_GLFW.get(key_str)
    if glfw_key is None:
        return

    if sys.platform == "win32":
        vk = _GLFW_TO_VK.get(glfw_key)
        if not vk:
            return
        WM_KEYDOWN, WM_KEYUP = 0x0100, 0x0101
        hwnd = _find_mujoco_hwnd()
        if hwnd:
            # フォーカス不要: PostMessageW でウィンドウのメッセージキューに直接挿入
            ctypes.windll.user32.PostMessageW(hwnd, WM_KEYDOWN, vk, 0)
            ctypes.windll.user32.PostMessageW(hwnd, WM_KEYUP,   vk, 0)
        else:
            # MuJoCo ウィンドウが見つからない場合は pyautogui フォールバック
            try:
                import pyautogui
                name = {265: "up", 264: "down", 263: "left", 262: "right", 32: "space"}
                pyautogui.press(name[glfw_key])
            except Exception as e:
                log.debug(f"pyautogui fallback: {e}")
    else:
        k = _GLFW_TO_XDO.get(glfw_key)
        if k:
            os.system(f"xdotool key --clearmodifiers {k}")


# ─── WebSocket ハンドラー ─────────────────────────────────────────────────────
async def ws_handler(request):
    ws = web.WebSocketResponse(heartbeat=30)
    await ws.prepare(request)
    _clients.add(ws)
    log.info(f"WS connected ({len(_clients)} total): {request.remote}")
    try:
        async for msg in ws:
            if msg.type == aiohttp.WSMsgType.TEXT:
                try:
                    data = json.loads(msg.data)
                    if data.get("type") == "key":
                        key = data.get("key", "")
                        loop = asyncio.get_running_loop()
                        await loop.run_in_executor(None, _inject_key_sync, key)
                except Exception as e:
                    log.debug(f"ws msg: {e}")
            elif msg.type in (aiohttp.WSMsgType.ERROR, aiohttp.WSMsgType.CLOSE):
                break
    finally:
        _clients.discard(ws)
        log.info(f"WS disconnected ({len(_clients)} remaining)")
    return ws


# ─── エントリポイント ─────────────────────────────────────────────────────────
async def main():
    ap = argparse.ArgumentParser(description="G1 Web UI Bridge Server")
    ap.add_argument("--port",            type=int, default=int(os.environ.get("WEBUI_PORT", 8765)))
    ap.add_argument("--camera-zmq-port", type=int, default=int(os.environ.get("CAMERA_ZMQ_PORT", 5555)))
    ap.add_argument("--launch-sim",      default="", help="Simulator launch command (stdout captured)")
    ap.add_argument("--launch-dialogue", default="", help="Dialogue system launch command (stdout captured)")
    ap.add_argument("--sim-log",         default="", help="Simulator log file to tail")
    ap.add_argument("--dialogue-log",    default="", help="Dialogue system log file to tail")
    args = ap.parse_args()

    logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")

    here = Path(__file__).parent
    app = web.Application()
    app.router.add_get("/ws", ws_handler)
    app.router.add_get("/", lambda r: web.FileResponse(here / "index.html"))
    app.router.add_static("/static", here / "static", show_index=False)

    loop = asyncio.get_running_loop()
    loop.create_task(camera_loop(args.camera_zmq_port))

    if args.launch_sim:
        loop.create_task(launch_subprocess("sim", args.launch_sim))
    elif args.sim_log:
        loop.create_task(tail_log("sim", args.sim_log))

    if args.launch_dialogue:
        loop.create_task(launch_subprocess("dialogue", args.launch_dialogue))
    elif args.dialogue_log:
        loop.create_task(tail_log("dialogue", args.dialogue_log))

    runner = web.AppRunner(app)
    await runner.setup()
    await web.TCPSite(runner, "0.0.0.0", args.port).start()
    log.info(f"Web UI → http://localhost:{args.port}")
    await asyncio.Event().wait()


if __name__ == "__main__":
    asyncio.run(main())
