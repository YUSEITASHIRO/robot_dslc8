#!/usr/bin/env python3
"""
g1_move_test_full.py — G1 Dense Grid Navigation (SONIC WSAD planner)

Coordinate system (robot-centric, origin = robot start):
  World frame   : degrees CCW from +x  (0° = forward = toward user at startup)
  SONIC planner : startup-relative       ([1,0,0] = robot startup direction = toward user)
  Conversion    : SONIC_rad = radians(world_deg)   ← same axis/origin

  Axes:  +x = forward (toward user)   +y = left   -y = right   -x = backward

Grid:
  Grid step = GRID_STEP_M (1.0m), origin at robot start (0,0).
  WBC step  = METERS_PER_STEP (~0.35m), calibrate with `walk N`.
  Safety margin = SAFETY_MARGIN (0.40m) from all walls and obstacles.
  Navigate: `go <x> <y>` snaps to nearest valid grid cell; BFS path planned.
  Consecutive same-direction steps are grouped to minimise turns.
"""

import json
import math
import struct
import threading
import time
from collections import deque
from typing import Optional

import zmq

# ─── Runtime settings ─────────────────────────────────────────────────────────
ZMQ_PORT             = 5556
HEADER_SIZE          = 1280
DEBUG_STEPS          = False    # toggle with `debug` CLI command

WALK_STEP_DURATION   = 0.35     # seconds per walk step (mode=2 hold time)
METERS_PER_STEP      = 0.35     # CALIBRATE: metres per single WBC step (measure with `walk N`)
GRID_STEP_M          = 1.00     # metres between grid circles (fixed layout)

SETTLE_TICK_S        = 0.05     # seconds between settle refreshes — must match original, WBC tuned for this

TURN_STEP_DEG        = 15.0     # degrees per discrete turn step
TURN_REPEAT_COUNT    = 2        # repeat each turn command N times
TURN_REPEAT_INTERVAL = 0.08     # seconds between repeated turn commands

TURN_SETTLE_S        = 1.2      # seconds of mode=2 zero-movement after a turn (feet repositioning)
FOOT_REALIGN_S       = 0.35     # brief walk after in-place turn to align feet (= 1 full WBC step cycle)

SAFETY_MARGIN        = 0.40     # metres from obstacle faces

# 方向名エイリアス (world degrees: 0°=前方/toward-user, 90°=左, -90°=右, 180°=後方)
_FACING_ALIASES: dict[str, float] = {
    "n":   0.0, "north":   0.0, "北":   0.0,   # forward  = toward user
    "s": 180.0, "south": 180.0, "南": 180.0,   # backward
    "e": -90.0, "east":  -90.0, "東": -90.0,   # right
    "w":  90.0, "west":   90.0, "西":  90.0,   # left
}

# ─── Room geometry (robot-centric coords: +x=forward, +y=left) ────────────────
# Transformed from scene_43dof.xml: new_x=old_y, new_y=3-old_x, hdx↔hdy.
# Each entry: (cx, cy, half_dx, half_dy).
# A grid cell is valid only if it is more than SAFETY_MARGIN away from every box.

_WALLS = [
    ( 0.0,  9.0,  2.0,  0.05),  # wall_back  (奥の壁)
    ( 0.0, -1.0,  2.0,  0.05),  # wall_front (入口の壁)
    (-2.0,  4.0,  0.05, 5.05),  # wall_right (right side)
    ( 2.0,  5.0,  0.05, 4.0 ),  # wall_left  (left side, y:1..9)
]

_OBSTACLES = [
    ( 1.78, 7.5,  0.2,  1.5  ),  # shelf_north_back   (left-far)
    ( 1.78, 3.5,  0.2,  2.5  ),  # shelf_north_mid    (left-mid)
    (-1.78, 7.5,  0.2,  1.5  ),  # shelf_south_back   (right-far)
    (-1.78, 3.5,  0.2,  2.5  ),  # shelf_south_mid    (right-mid)
    (-1.78, 0.0,  0.2,  1.0  ),  # shelf_south_east   (right-near, behind robot)
    ( 1.5,  1.0,  0.5,  0.05 ),  # wall_half_partition (shortened, x=1.0..2.0)
    ( 1.5,  1.25, 0.4,  0.2  ),  # shelf_partition_side (shortened to match)
    (-0.5,  7.8,  0.9,  0.7  ),  # counter_table      (奥)
    ( 0.65, 7.8,  0.2,  0.65 ),  # register_counter   (奥)
]

_ALL_BOXES = _WALLS + _OBSTACLES


def _is_safe(x: float, y: float) -> bool:
    """True if (x,y) is at least SAFETY_MARGIN away from every wall and obstacle."""
    for cx, cy, hdx, hdy in _ALL_BOXES:
        if abs(x - cx) < hdx + SAFETY_MARGIN and abs(y - cy) < hdy + SAFETY_MARGIN:
            return False
    return True


# ─── Grid generation ──────────────────────────────────────────────────────────

def build_grid() -> dict:
    """Build dict: (xi, yi) → (x, y) for all valid grid cells.
    Grid origin at robot start (0,0); cell spacing = GRID_STEP_M.
    Room bounds in robot-centric coords: x∈[-2,2]  y∈[-1,9].
    """
    step = GRID_STEP_M
    xi_min = math.ceil( (-2.0 + SAFETY_MARGIN) / step)   # left/right walls at ±2
    xi_max = math.floor(( 2.0 - SAFETY_MARGIN) / step)
    yi_min = math.ceil( (-1.0 + SAFETY_MARGIN) / step)   # entrance wall at y=-1
    yi_max = math.floor(( 9.0 - SAFETY_MARGIN) / step)   # back wall at y=9

    grid: dict = {}
    for xi in range(xi_min, xi_max + 1):
        for yi in range(yi_min, yi_max + 1):
            x = round(xi * step, 6)
            y = round(yi * step, 6)
            if _is_safe(x, y):
                grid[(xi, yi)] = (x, y)
    return grid


# ─── Pathfinding (4-directional BFS) ──────────────────────────────────────────

_DIRS4 = [(1, 0), (-1, 0), (0, 1), (0, -1)]  # E W N S


def bfs_path(grid: dict, start: tuple, end: tuple) -> Optional[list]:
    """BFS shortest path (list of (xi,yi)) from start to end on grid."""
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


def group_path(path: list) -> list:
    """Group consecutive same-direction steps → [((dx,dy), count), ...]."""
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


def plan_minimal_turns(grid: dict, start: tuple, end: tuple) -> Optional[list]:
    """Plan from start to end with fewest direction changes.

    Tries a direct L-shaped path (X-then-Y or Y-then-X) first — at most 1 turn.
    Falls back to BFS only when the direct route is blocked by an obstacle.
    Returns [((dx,dy), count), ...] same format as group_path().
    """
    if start == end:
        return []

    xi0, yi0 = start
    xi1, yi1 = end
    dx      = (1 if xi1 > xi0 else -1) if xi1 != xi0 else 0
    dy      = (1 if yi1 > yi0 else -1) if yi1 != yi0 else 0
    steps_x = abs(xi1 - xi0)
    steps_y = abs(yi1 - yi0)

    def clear(ox: int, oy: int, ddx: int, ddy: int, n: int) -> bool:
        for i in range(1, n + 1):
            if (ox + ddx * i, oy + ddy * i) not in grid:
                return False
        return True

    # Option A: walk X first, then Y  (1 turn if both non-zero)
    x_ok = steps_x == 0 or clear(xi0, yi0, dx, 0, steps_x)
    y_ok = steps_y == 0 or clear(xi1, yi0, 0, dy, steps_y)
    if x_ok and y_ok:
        segs = []
        if steps_x: segs.append(((dx, 0), steps_x))
        if steps_y: segs.append(((0, dy), steps_y))
        return segs

    # Option B: walk Y first, then X
    y_ok2 = steps_y == 0 or clear(xi0, yi0, 0, dy, steps_y)
    x_ok2 = steps_x == 0 or clear(xi0, yi1, dx, 0, steps_x)
    if y_ok2 and x_ok2:
        segs = []
        if steps_y: segs.append(((0, dy), steps_y))
        if steps_x: segs.append(((dx, 0), steps_x))
        return segs

    # Fallback: BFS (obstacle in the way — e.g. partition wall)
    path = bfs_path(grid, start, end)
    return group_path(path) if path else None


def nearest_grid(grid: dict, x: float, y: float) -> Optional[tuple]:
    """Return grid cell (xi, yi) whose world position is closest to (x, y)."""
    best, best_d = None, math.inf
    for (xi, yi), (gx, gy) in grid.items():
        d = math.hypot(x - gx, y - gy)
        if d < best_d:
            best_d, best = d, (xi, yi)
    return best


def check_connectivity(grid: dict, start: tuple) -> set:
    """BFS flood-fill from start; return set of reachable (xi,yi)."""
    if start not in grid:
        return set()
    reachable = {start}
    q: deque = deque([start])
    while q:
        pos = q.popleft()
        for dx, dy in _DIRS4:
            nxt = (pos[0] + dx, pos[1] + dy)
            if nxt not in reachable and nxt in grid:
                reachable.add(nxt)
                q.append(nxt)
    return reachable


# ─── Angle helpers ────────────────────────────────────────────────────────────

def _m2s(deg: float) -> float:
    """World degrees → SONIC radians.  0° = forward (toward user) = [1,0,0]."""
    return math.radians(deg)


def _norm(a: float) -> float:
    while a >  math.pi: a -= 2 * math.pi
    while a <= -math.pi: a += 2 * math.pi
    return a


def _dir_to_sonic(dx: int, dy: int) -> float:
    """Grid step direction (dx,dy) → SONIC travel angle (radians).
    (1,0)=forward=0  (0,1)=left=π/2  (-1,0)=back=π  (0,-1)=right=-π/2
    """
    return math.atan2(dy, dx)


# ─── WalkerController ─────────────────────────────────────────────────────────

class WalkerController:
    def __init__(self, sock):
        self._sock         = sock
        self._facing_angle = 0.0
        self._TURN_STEP    = math.radians(TURN_STEP_DEG)
        self._planner_mode = False
        self._lock         = threading.Lock()
        self._action_stop  = threading.Event()
        self._action_thread: Optional[threading.Thread] = None

    # ── ZMQ send ──────────────────────────────────────────────────
    def _send_msg(self, topic: bytes, fields: list, data: bytes):
        hj = json.dumps({"v": 1, "endian": "le", "count": 1, "fields": fields}).encode()
        hb = hj + b"\x00" * (HEADER_SIZE - len(hj))
        self._sock.send(topic + hb + data)

    def send_command(self, start=True, stop=False, planner=True):
        fields = [{"name": "start",   "dtype": "u8", "shape": [1]},
                  {"name": "stop",    "dtype": "u8", "shape": [1]},
                  {"name": "planner", "dtype": "u8", "shape": [1]}]
        self._send_msg(b"command", fields, struct.pack("BBB", int(start), int(stop), int(planner)))

    def send_planner(self, mode: int, movement: list, facing: list, speed: float = -1.0):
        fields = [{"name": "mode",     "dtype": "i32", "shape": [1]},
                  {"name": "movement", "dtype": "f32", "shape": [3]},
                  {"name": "facing",   "dtype": "f32", "shape": [3]},
                  {"name": "speed",    "dtype": "f32", "shape": [1]},
                  {"name": "height",   "dtype": "f32", "shape": [1]}]
        data  = struct.pack("<i",   mode)
        data += struct.pack("<fff", *movement)
        data += struct.pack("<fff", *facing)
        data += struct.pack("<ff",  speed, -1.0)
        self._send_msg(b"planner", fields, data)

    # ── Internal helpers ───────────────────────────────────────────
    def _fv(self) -> list:
        a = self._facing_angle
        return [math.cos(a), math.sin(a), 0.0]

    def _wait_or_stop(self, seconds: float) -> bool:
        return self._action_stop.wait(timeout=seconds)

    def _cancel_action(self, wait: bool = True):
        self._action_stop.set()
        if wait and self._action_thread and self._action_thread.is_alive():
            self._action_thread.join(timeout=1.0)
        self._action_thread = None
        self._action_stop.clear()

    def _ensure_planner(self) -> bool:
        if not self._planner_mode:
            self.send_command(start=True, stop=False, planner=True)
            if self._wait_or_stop(0.3):
                return False
            self._planner_mode = True
        return True

    # ── High-level primitives ──────────────────────────────────────
    def start_planner(self):
        """Enter SONIC planner mode; facing=[1,0,0] = toward user = no rotation."""
        with self._lock:
            if not self._planner_mode:
                self.send_command(start=True, stop=False, planner=True)
                if self._wait_or_stop(1.0):
                    return
                self.send_planner(0, [0, 0, 0], self._fv())
                self._planner_mode = True
                print(f"[Walker] planner モード開始  facing={math.degrees(self._facing_angle):.0f}°(SONIC)")
            else:
                print("[Walker] 既にplanner モード")

    def settle_facing(self, settle_s: float = TURN_SETTLE_S) -> bool:
        """After a turn, send mode=2 zero-movement so SONIC repositions feet to a stable stance.
        Must use SETTLE_TICK_S (0.05s) — slower rates cause WBC instability during stance repositioning.
        """
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
        """After an in-place turn, send a brief walk so WBC steps both feet into alignment."""
        fv = self._fv()
        self.send_planner(2, fv, fv)
        if self._wait_or_stop(FOOT_REALIGN_S):
            self.send_planner(0, [0, 0, 0], fv)
            return False
        self.send_planner(0, [0, 0, 0], fv)
        return True

    def walk_forward(self, steps: int = 1) -> bool:
        """Walk for N steps using the same cadence as WSAD _walk_linear:
        mode=2 (WALK_STEP_DURATION) → mode=0 → 0.08s gap → repeat.
        The inter-step gap resets WBC lean so tilt does not accumulate.
        """
        steps = max(1, int(steps))
        with self._lock:
            if not self._ensure_planner():
                return False
            a  = self._facing_angle
            mv = [math.cos(a), math.sin(a), 0.0]
            t0 = time.time()
            for i in range(steps):
                if DEBUG_STEPS:
                    print(f"    [step {i+1}/{steps}] t={time.time()-t0:.3f}s  "
                          f"mode=2 mv={[round(v,2) for v in mv]}", flush=True)
                self.send_planner(2, mv, self._fv())
                if self._wait_or_stop(WALK_STEP_DURATION):
                    self.send_planner(0, [0, 0, 0], self._fv())
                    return False
                self.send_planner(0, [0, 0, 0], self._fv())
                if DEBUG_STEPS:
                    print(f"    [step {i+1}/{steps}] t={time.time()-t0:.3f}s  "
                          f"mode=0  gap→", flush=True)
                if i < steps - 1:
                    if self._wait_or_stop(0.08):
                        return False
            if DEBUG_STEPS:
                print(f"    [walk done] total={time.time()-t0:.3f}s  "
                      f"est={steps*METERS_PER_STEP:.2f}m", flush=True)
        return True

    def turn_left(self, steps: int = 1) -> bool:
        with self._lock:
            if not self._ensure_planner():
                return False
            n = max(1, int(steps))
            for i in range(n):
                self._facing_angle += self._TURN_STEP
                fv = self._fv()
                for _ in range(TURN_REPEAT_COUNT):
                    self.send_planner(2, [0, 0, 0], fv)
                    if self._wait_or_stop(TURN_REPEAT_INTERVAL):
                        self.send_planner(0, [0, 0, 0], self._fv())
                        return False
                if i < n - 1:
                    self.send_planner(0, [0, 0, 0], fv)
                    if self._wait_or_stop(TURN_REPEAT_INTERVAL):
                        return False
            self.send_planner(0, [0, 0, 0], self._fv())
        return True

    def turn_right(self, steps: int = 1) -> bool:
        with self._lock:
            if not self._ensure_planner():
                return False
            n = max(1, int(steps))
            for i in range(n):
                self._facing_angle -= self._TURN_STEP
                fv = self._fv()
                for _ in range(TURN_REPEAT_COUNT):
                    self.send_planner(2, [0, 0, 0], fv)
                    if self._wait_or_stop(TURN_REPEAT_INTERVAL):
                        self.send_planner(0, [0, 0, 0], self._fv())
                        return False
                if i < n - 1:
                    self.send_planner(0, [0, 0, 0], fv)
                    if self._wait_or_stop(TURN_REPEAT_INTERVAL):
                        return False
            self.send_planner(0, [0, 0, 0], self._fv())
        return True

    def stop(self):
        self._cancel_action(wait=False)
        with self._lock:
            if self._planner_mode:
                self.send_planner(0, [0, 0, 0], self._fv())


# ─── GridNavigator ────────────────────────────────────────────────────────────

class GridNavigator:
    def __init__(self, walker: WalkerController, grid: dict,
                 start_xi: int, start_yi: int):
        self._walker = walker
        self._grid   = grid
        self._xi     = start_xi
        self._yi     = start_yi
        # Startup facing: forward (toward user) = world 0° = SONIC 0 → [1,0,0]
        walker._facing_angle = _m2s(0.0)

    @property
    def cell(self) -> tuple:
        return (self._xi, self._yi)

    @property
    def world_pos(self) -> tuple:
        return self._grid.get((self._xi, self._yi),
                              (round(self._xi * GRID_STEP_M, 3),
                               round(self._yi * GRID_STEP_M, 3)))

    def face_direction(self, target_deg: float) -> bool:
        """Turn in place to face the given world angle (degrees, 0°=forward)."""
        target_sonic = _m2s(target_deg)
        w = self._walker
        delta = _norm(target_sonic - w._facing_angle)
        n = round(abs(delta) / math.radians(TURN_STEP_DEG))
        if n == 0:
            return True
        tdir = "left" if delta >= 0 else "right"
        print(f"  [向き] {'左' if tdir=='left' else '右'} {n*TURN_STEP_DEG:.0f}°  "
              f"→ {target_deg:.0f}°  足位置調整...", flush=True)
        ok = w.turn_left(n) if tdir == "left" else w.turn_right(n)
        if not ok:
            return False
        if not w.settle_facing():
            return False
        return w.micro_align()

    def goto(self, xi: int, yi: int,
             facing_deg: Optional[float] = None,
             dry_run: bool = False) -> bool:
        if (xi, yi) not in self._grid:
            print(f"[Nav] ({xi},{yi}) はグリッド外または障害物内")
            return False

        _DLABEL = {(1,0):'前',(-1,0):'後',(0,1):'左',(0,-1):'右'}
        _FNAME  = {0.0:'前(北)', 180.0:'後(南)', -180.0:'後(南)', -90.0:'右(東)', 90.0:'左(西)'}

        # ── 移動フェーズ ──────────────────────────────────────────
        if (xi, yi) != (self._xi, self._yi):
            segs = plan_minimal_turns(self._grid, (self._xi, self._yi), (xi, yi))
            if segs is None:
                print(f"[Nav] 経路なし: ({self._xi},{self._yi}) → ({xi},{yi})")
                return False

            total  = sum(c for _, c in segs)
            turns  = len(segs) - 1
            tx, ty = self._grid[(xi, yi)]
            detour = turns >= 2
            face_str = (f"  → 向き:{_FNAME.get(facing_deg, f'{facing_deg:.0f}°')}"
                        if facing_deg is not None else "")
            print(f"\n[Nav] → ({xi},{yi}) ({tx:.2f},{ty:.2f})"
                  f"  {total*GRID_STEP_M:.2f}m  旋回={turns}"
                  + ("  ※迂回" if detour else "") + face_str)
            for (dx, dy), cnt in segs:
                print(f"      {_DLABEL.get((dx,dy),'?')} ×{cnt} ({cnt*GRID_STEP_M:.2f}m)")

            if not dry_run and not self._execute(segs):
                return False
        else:
            wx, wy = self.world_pos
            print(f"[Nav] 既にその位置 ({xi},{yi}) ({wx:.2f},{wy:.2f})")

        # ── 向きフェーズ ──────────────────────────────────────────
        if facing_deg is not None:
            fname = _FNAME.get(facing_deg, f"{facing_deg:.0f}°")
            print(f"  [向き] → {fname} (MuJoCo {facing_deg:.0f}°)")
            if not dry_run:
                return self.face_direction(facing_deg)

        return True

    def _execute(self, segs: list) -> bool:
        w = self._walker
        for seg_idx, ((dx, dy), cnt) in enumerate(segs):
            label = f"[{seg_idx+1}/{len(segs)}]"
            travel_sonic = _dir_to_sonic(dx, dy)
            delta = _norm(travel_sonic - w._facing_angle)
            n     = round(abs(delta) / math.radians(TURN_STEP_DEG))
            tdir  = "left" if delta >= 0 else "right"

            if n > 0:
                print(f"  {label} 旋回{'左' if tdir=='left' else '右'} "
                      f"{n * TURN_STEP_DEG:.0f}°  足位置調整...", flush=True)
                ok = w.turn_left(n) if tdir == "left" else w.turn_right(n)
                if not ok:
                    print("  ─ 中断 ─")
                    return False
                if not w.settle_facing():
                    print("  ─ 中断 ─")
                    return False

            wbc_steps = max(1, round(cnt * GRID_STEP_M / METERS_PER_STEP))
            print(f"  {label} 前進 {cnt}格子 ({cnt * GRID_STEP_M:.2f}m) → {wbc_steps}ステップ", flush=True)
            if not w.walk_forward(wbc_steps):
                print("  ─ 中断 ─")
                return False

            # Update dead-reckoning
            self._xi += dx * cnt
            self._yi += dy * cnt

        # Stabilise after final walk step: mode=2 zero-movement lets WBC absorb
        # residual forward lean without uncontrolled balance steps.
        w.settle_facing(settle_s=0.5)

        wx, wy = self.world_pos
        print(f"  ✓ 完了 → ({self._xi},{self._yi}) ({wx:.2f},{wy:.2f})")
        return True

    def reset_to(self, xi: int, yi: int):
        if (xi, yi) not in self._grid:
            print(f"[Nav] ({xi},{yi}) はグリッド外")
            return
        self._xi, self._yi = xi, yi
        self._walker._facing_angle = _m2s(0.0)
        wx, wy = self.world_pos
        print(f"[Nav] リセット → ({xi},{yi}) ({wx:.2f},{wy:.2f})")


# ─── Grid display ─────────────────────────────────────────────────────────────

def print_grid(grid: dict, cur_xi: int, cur_yi: int):
    if not grid:
        print("  (空のグリッド)")
        return
    xs = [xi for xi, _ in grid]
    ys = [yi for _, yi in grid]
    xi_min, xi_max = min(xs), max(xs)
    yi_min, yi_max = min(ys), max(ys)

    print(f"\n── Grid  step={GRID_STEP_M:.2f}m  mps={METERS_PER_STEP:.3f}m  margin={SAFETY_MARGIN:.2f}m  cells={len(grid)} ──")
    for yi in range(yi_max, yi_min - 1, -1):
        row = ""
        for xi in range(xi_min, xi_max + 1):
            if xi == cur_xi and yi == cur_yi:
                row += "R"
            elif (xi, yi) in grid:
                row += "."
            else:
                row += " "
        yv = yi * GRID_STEP_M
        print(f"  y={yv:+.2f} |{row}|")
    xv_min = xi_min * GRID_STEP_M
    xv_max = xi_max * GRID_STEP_M
    print(f"           x: {xv_min:+.2f} .. {xv_max:+.2f}")


def _print_scene_map(grid: dict) -> None:
    """Print a top-down ASCII map of the store with coordinate labels.

    Orientation:
      x → right  = N/前方 (+x, toward user)
      y ↑ up     = W/左方 (+y, deeper into store)
      Entrance opening: right side (x=+2 gap at y=-1..0)
    """
    X_VALS = [-2, -1,  0,  1,  2]
    Y_VALS = list(range(9, -2, -1))   # 9 (top/back) → -1 (bottom/front)

    m: dict[tuple[int, int], str] = {}

    # ── shelves along right wall (x=-2, full y) and left wall (x=+2, y≥2)
    for yi in range(-1, 9):  m[(-2, yi)] = '#'
    for yi in range( 2, 9):  m[( 2, yi)] = '#'
    # ── counter area (~y=8)
    for xi in (-1, 0, 1):    m[(xi,  8)] = 'C'
    # ── partition wall tip at (1,1)
    m[(1, 1)] = '-'

    # ── valid grid cells (only if not already an obstacle mark)
    for xi, yi in grid:
        if (xi, yi) not in m:
            m[(xi, yi)] = 'o'

    # ── robot start & user position
    m[( 0, 0)] = 'R'
    m[( 2, 0)] = 'U'   # user at x=1.5 → shown in x=+2 column

    # ── build display ──────────────────────────────────────────
    CW = 4  # display chars per cell (header width)
    IW = 3  # actual inner chars per cell: ' X '

    def cell_ch(x, y) -> str:
        return f' {m.get((x, y), " "):1} '   # IW = 3 chars

    def row_line(y: int) -> str:
        inner = ''.join(cell_ch(x, y) for x in X_VALS)   # 5×3 = 15 chars
        lw = '|'                                           # right wall (x=-2 side)
        rw = '|' if y >= 1 else ' '                        # left wall gap at y≤0
        return f'{lw}{inner}{rw}'

    n_inner = len(X_VALS) * IW                             # 15
    hdr_x   = '       ' + ''.join(f'{x:^{CW}}' for x in X_VALS)
    bar_top = '       +' + '=' * n_inner + '+  <- 奥の壁 (y=9)'
    bar_bot = '       +' + '=' * n_inner + '+  <- 前面壁 (y=-1)'

    notes = {
        8: '<- カウンター',
        1: '<- 仕切り壁',
        0: '<- 起動位置 / 入口開口 (右側)',
    }

    compass = [
        '    W(+y,左,奥)  ',
        '        ^        ',
        '  S(-x) | N(+x)  ',
        '  (後)<-+->(前)  ',
        '        v        ',
        '    E(-y,右,入口) ',
    ]

    lines: list[str] = []
    lines.append('')
    lines.append('━' * 52)
    lines.append('  店舗マップ (俯瞰)  起動時スナップショット')
    lines.append('━' * 52)
    lines.append(hdr_x + '    x 軸')
    lines.append(bar_top)
    for i, y in enumerate(Y_VALS):
        label   = f' y={y:+2d} '
        note    = notes.get(y, '')
        cmp     = compass[i] if i < len(compass) else ''
        lines.append(label + row_line(y) + f'  {note:<30}{cmp}')
    lines.append(bar_bot)
    lines.append('')
    lines.append('  凡例:  o 有効格子  R 起動位置  U ユーザー')
    lines.append('         # 壁/棚    C カウンター  - 仕切り端')
    lines.append('━' * 52)

    for ln in lines:
        print(ln)


# ─── Interactive CLI ──────────────────────────────────────────────────────────

_HELP = """\
─────────────────────────────────────────────────────
  G1 ナビゲーション コマンド一覧
─────────────────────────────────────────────────────
【移動】
  go <x> <y>              (x,y) に最も近いグリッドへ移動
                            例: go 0 0
  go <x> <y> <向き>       移動後に指定方向へ旋回
                            例: go 0 0 n        (原点へ移動、北向き)
                            例: go -2 0.9 90    (移動後 90° 向き)
  go <xi> <yi> i          グリッドインデックスで直接指定
                            例: go 0 0 i
  dry go <x> <y> [向き]   経路確認のみ、実際には動かない
                            例: dry go -3 0

【向き転換 (その場)】
  face <向き>             その場で方向転換
                            例: face n          (北向き／ユーザー方向)
                            例: face s          (南向き)
                            例: face e          (東向き)
                            例: face w          (西向き)
                            例: face 90         (MuJoCo 度数で指定)

【校正】
  walk <n>                N ステップ直進 (実測距離の確認用)
                            例: walk 5
  mps <値>                METERS_PER_STEP を変更
                            例: mps 0.28

【その他】
  check                   現在位置から全セルの到達可否を確認
  grid / g                ASCII グリッドマップ表示 (@ = ロボット位置)
  where / w               現在位置と向きを表示
  status / s              位置・向き・校正値を表示
  reset <x> <y>           デッドレコニング位置リセット (物理移動なし)
                            例: reset 0 0
  debug                   ステップごとのタイミング表示 ON/OFF (精度確認用)
  start                   SONIC planner モード開始
  stop                    動作キャンセル
  q / quit                終了
  h / ?                   このヘルプを表示

【向き指定】
  n / north / 北 =   0°  (前方=ユーザー方向、ロボット起動向き)
  s / south / 南 = 180°  (後方)
  e / east  / 東 = -90°  (右)
  w / west  / 西 =  90°  (左)
─────────────────────────────────────────────────────"""


def _parse_facing(token: str) -> Optional[float]:
    """Parse a facing token: alias name or numeric degrees. Returns None if invalid."""
    low = token.lower()
    if low in _FACING_ALIASES:
        return _FACING_ALIASES[low]
    try:
        return float(token)
    except ValueError:
        return None


def _handle_go(args: list, nav: GridNavigator, grid: dict, dry_run: bool):
    if not args:
        print("  使い方: go <x> <y> [facing]  例: go 0 0 n")
        return

    # ── 座標指定 ──────────────────────────────────────────────
    try:
        facing: Optional[float] = None

        if len(args) >= 3 and args[2].lower() == "i":
            # グリッドインデックス指定: go xi yi i [facing]
            xi, yi = int(args[0]), int(args[1])
            if len(args) >= 4:
                facing = _parse_facing(args[3])
        else:
            # ワールド座標指定: go x y [facing]
            wx, wy = float(args[0]), float(args[1])
            cell = nearest_grid(grid, wx, wy)
            if cell is None:
                print("  有効な格子が見つかりません")
                return
            xi, yi = cell
            gx, gy = grid[cell]
            print(f"  スナップ → ({xi},{yi}) ({gx:.2f},{gy:.2f})")
            if len(args) >= 3:
                facing = _parse_facing(args[2])
                if facing is None:
                    print(f"  ⚠ facing '{args[2]}' は無効  (n/s/e/w または度数)")

        nav.goto(xi, yi, facing_deg=facing, dry_run=dry_run)

    except (ValueError, IndexError):
        print("  使い方: go <x> <y> [facing]  例: go 0 0 n  / go -2 0.9 90")


def run_cli(walker: WalkerController, nav: GridNavigator, grid: dict):
    global METERS_PER_STEP, TURN_SETTLE_S

    _print_scene_map(grid)
    print(_HELP)
    xi, yi = nav.cell
    wx, wy = nav.world_pos
    print(f"\n開始位置: ({xi},{yi}) ({wx:.2f},{wy:.2f})  有効格子数={len(grid)}")
    print("grid でマップ表示  start で planner 起動")

    while True:
        xi, yi = nav.cell
        wx, wy = nav.world_pos
        try:
            raw = input(f"\n[({xi},{yi}) ({wx:.2f},{wy:.2f})]> ").strip()
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

        elif cmd == "face":
            if len(parts) < 2:
                print("  使い方: face <dir>  例: face n  face w  face 90")
            else:
                facing = _parse_facing(parts[1])
                if facing is None:
                    print(f"  ⚠ 不明な方向: {parts[1]}  (n/s/e/w または度数)")
                else:
                    _FNAME = {0.0:'前(北)', 180.0:'後(南)', -180.0:'後(南)', -90.0:'右(東)', 90.0:'左(西)'}
                    print(f"  向き → {_FNAME.get(facing, f'{facing:.0f}°')} ({facing:.0f}°)")
                    nav.face_direction(facing)

        elif cmd == "check":
            rc = check_connectivity(grid, nav.cell)
            isolated = [c for c in grid if c not in rc]
            if isolated:
                print(f"  ⚠ 到達不可: {len(isolated)}セル")
                for c in isolated:
                    print(f"    {c} {grid[c]}")
            else:
                print(f"  OK: 現在位置から全{len(grid)}セル到達可能")

        elif cmd in ("grid", "g"):
            print_grid(grid, nav.cell[0], nav.cell[1])

        elif cmd in ("where", "w"):
            deg = math.degrees(walker._facing_angle)
            xi, yi = nav.cell
            wx, wy = nav.world_pos
            _FNAME = {0.0:'前(北)', 180.0:'後(南)', -180.0:'後(南)', -90.0:'右(東)', 90.0:'左(西)'}
            print(f"  格子 ({xi},{yi})  ({wx:.2f},{wy:.2f})")
            print(f"  向き: {deg:.1f}°  ({_FNAME.get(round(deg), f'{deg:.0f}°')})")

        elif cmd in ("status", "s"):
            deg = math.degrees(walker._facing_angle)
            xi, yi = nav.cell
            wx, wy = nav.world_pos
            _FNAME = {0.0:'前(北)', 180.0:'後(南)', -180.0:'後(南)', -90.0:'右(東)', 90.0:'左(西)'}
            print(f"  格子 ({xi},{yi})  ({wx:.2f},{wy:.2f})")
            print(f"  向き: {deg:.1f}°  ({_FNAME.get(round(deg), f'{deg:.0f}°')})")
            print(f"  GRID_STEP_M={GRID_STEP_M:.2f}m  METERS_PER_STEP={METERS_PER_STEP:.3f}m  WALK_STEP_DURATION={WALK_STEP_DURATION}s")
            print(f"  TURN_SETTLE_S={TURN_SETTLE_S}s  valid_cells={len(grid)}")

        elif cmd == "walk":
            if len(parts) < 2 or not parts[1].lstrip('-').isdigit():
                print("  使い方: walk <ステップ数>")
            else:
                n = int(parts[1])
                if n <= 0:
                    print("  ステップ数は1以上")
                else:
                    print(f"  前進 {n}ステップ  推定 {n * METERS_PER_STEP:.3f}m  (METERS_PER_STEP={METERS_PER_STEP:.3f})")
                    print(f"  実測距離 ÷ {n} = 正しい METERS_PER_STEP 値  (mps コマンドで更新)")
                    walker.walk_forward(n)

        elif cmd == "mps":
            if len(parts) < 2:
                print(f"  現在の METERS_PER_STEP = {METERS_PER_STEP}")
            else:
                try:
                    val = float(parts[1])
                    if val <= 0:
                        raise ValueError
                    METERS_PER_STEP = val
                    print(f"  METERS_PER_STEP = {METERS_PER_STEP}")
                    print("  注: 格子サイズは起動時固定。完全反映には再起動が必要。")
                except ValueError:
                    print("  数値を入力してください")

        elif cmd == "start":
            walker.start_planner()

        elif cmd == "stop":
            walker.stop()

        elif cmd == "reset":
            if len(parts) < 3:
                print("  使い方: reset <x> <y>")
            else:
                try:
                    rx, ry = float(parts[1]), float(parts[2])
                    cell = nearest_grid(grid, rx, ry)
                    if cell:
                        nav.reset_to(*cell)
                    else:
                        print("  有効な格子が見つかりません")
                except ValueError:
                    print("  数値を入力してください")

        elif cmd == "dry" and len(parts) >= 3 and parts[1].lower() == "go":
            _handle_go(parts[2:], nav, grid, dry_run=True)

        elif cmd == "go":
            if len(parts) < 2:
                print("  使い方: go <x> <y> [facing]  例: go 0 0 n")
            else:
                _handle_go(parts[1:], nav, grid, dry_run=False)

        elif cmd == "debug":
            global DEBUG_STEPS
            DEBUG_STEPS = not DEBUG_STEPS
            state = "ON" if DEBUG_STEPS else "OFF"
            print(f"  デバッグ出力: {state}")
            if DEBUG_STEPS:
                print(f"  各ステップのタイミングを表示します")
                print(f"  WALK_STEP_DURATION={WALK_STEP_DURATION}s  gap=0.08s")
                print(f"  1格子={GRID_STEP_M:.2f}m → {round(GRID_STEP_M/METERS_PER_STEP):.0f}ステップ  "
                      f"(METERS_PER_STEP={METERS_PER_STEP:.3f}m)")

        else:
            print(f"  不明なコマンド: {raw}  (h でヘルプ)")


# ─── Entry point ──────────────────────────────────────────────────────────────

def main():
    global METERS_PER_STEP, TURN_SETTLE_S

    import argparse
    ap = argparse.ArgumentParser(description="G1 Dense Grid Navigation (SONIC WSAD planner)")
    ap.add_argument("--zmq-port", type=int, default=ZMQ_PORT)
    ap.add_argument("--start", nargs=2, type=float, default=[0.0, 0.0],
                    metavar=("X", "Y"),
                    help="Start position in robot-centric coords (default: 0.0 0.0)")
    ap.add_argument("--mps", type=float, default=None,
                    help=f"Metres per step (default: {METERS_PER_STEP})")
    ap.add_argument("--settle", type=float, default=None,
                    help=f"Turn settle seconds (default: {TURN_SETTLE_S})")
    args = ap.parse_args()

    if args.mps    is not None: METERS_PER_STEP = args.mps
    if args.settle is not None: TURN_SETTLE_S   = args.settle

    grid = build_grid()
    print(f"グリッド生成完了: {len(grid)} 有効格子  grid_step={GRID_STEP_M:.2f}m  mps={METERS_PER_STEP:.3f}m  margin={SAFETY_MARGIN:.2f}m")

    sx, sy = args.start
    start_cell = nearest_grid(grid, sx, sy)
    if start_cell is None:
        print(f"エラー: 開始位置 ({sx},{sy}) の近くに有効格子なし")
        return
    start_xi, start_yi = start_cell
    gx, gy = grid[start_cell]
    print(f"開始格子: ({start_xi},{start_yi}) ({gx:.2f},{gy:.2f})")

    # 連結確認: 全セルが開始点から到達可能か検証
    reachable = check_connectivity(grid, start_cell)
    isolated  = [c for c in grid if c not in reachable]
    if isolated:
        print(f"⚠ 孤立格子 {len(isolated)} セル (到達不可):")
        for c in isolated:
            print(f"   {c} {grid[c]}")
    else:
        print(f"連結確認OK: 全{len(grid)}セル到達可能 (BFS迂回ルーティング有効)")

    ctx  = zmq.Context()
    sock = ctx.socket(zmq.PUB)
    sock.bind(f"tcp://*:{args.zmq_port}")
    time.sleep(0.3)

    walker = WalkerController(sock)
    nav    = GridNavigator(walker, grid, start_xi, start_yi)

    print(f"ZMQ PUB port={args.zmq_port}  mps={METERS_PER_STEP}  settle={TURN_SETTLE_S}s")

    # Trigger WBC; facing=[1,0,0] = toward user = no startup rotation.
    walker.start_planner()

    try:
        run_cli(walker, nav, grid)
    finally:
        walker.stop()
        sock.close()
        ctx.term()
        print("[終了]")


if __name__ == "__main__":
    main()
