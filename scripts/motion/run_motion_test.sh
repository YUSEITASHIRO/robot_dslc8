#!/bin/bash
# run_motion_test.sh — G1 動作テスト CLI 起動スクリプト
# run_sim.sh + run_deploy.sh 起動後に実行してください

REPO_ROOT="$(cd "$(dirname "$0")/../.." && pwd)"

conda activate g1_deploy 2>/dev/null || true

python "$REPO_ROOT/src/motion/g1_motion_test.py" "$@"
