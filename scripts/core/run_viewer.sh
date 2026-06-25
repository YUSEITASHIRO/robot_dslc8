#!/bin/bash
# MuJoCo カメラ映像をウィンドウ表示する（run_sim.sh 起動後に実行）

REPO_ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
EXTERN="$REPO_ROOT/extern/GR00T-WholeBodyControl"

if [ -d "$EXTERN/.venv_sim" ]; then
    source "$EXTERN/.venv_sim/bin/activate"
fi

cd "$REPO_ROOT"
python src/dialogue_system/mujoco_viewer.py "$@"
