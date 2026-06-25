#!/bin/bash
# run_simple_walk.sh — WASD キーボードで G1 を手動操作
#
# 事前準備:
#   Terminal 1: bash scripts/core/run_sim.sh
#   Terminal 2: bash scripts/core/run_deploy.sh
#   Terminal 3: このスクリプト
#
# 操作: W=前進  S=後退  A=左旋回  D=右旋回  Q=左横移動  E=右横移動  Space=停止  X=終了

REPO_ROOT="$(cd "$(dirname "$0")/../.." && pwd)"

conda activate g1_deploy 2>/dev/null || true

python "$REPO_ROOT/src/move/g1_simple_walk.py" "$@"
