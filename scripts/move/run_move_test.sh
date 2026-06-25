#!/bin/bash
# run_move_test.sh — グリッドナビゲーション対話 CLI
#
# 事前準備:
#   Terminal 1: bash scripts/core/run_sim.sh
#   Terminal 2: bash scripts/core/run_deploy.sh
#   Terminal 3: このスクリプト
#
# Usage:
#   bash scripts/run_move_test.sh
#   bash scripts/run_move_test.sh --dry-run   # 実際には動かさずパスだけ確認

REPO_ROOT="$(cd "$(dirname "$0")/../.." && pwd)"

conda activate g1_deploy 2>/dev/null || true

python "$REPO_ROOT/src/move/g1_move_test.py" "$@"
