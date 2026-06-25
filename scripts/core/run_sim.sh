#!/bin/bash
# MuJoCo シミュレーターを起動するラッパー

REPO_ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
EXTERN="$REPO_ROOT/extern/GR00T-WholeBodyControl"

VENV_SIM="$EXTERN/.venv_sim"
PYTHON="$VENV_SIM/bin/python"

# venv_sim が書き込み不可（他ユーザーの venv をシンボリックリンク）でも
# Python バイナリを直接指定することで動作可能
if [ ! -f "$PYTHON" ]; then
    echo "エラー: $PYTHON が見つかりません"
    echo "extern/GR00T-WholeBodyControl/.venv_sim が正しく設定されているか確認してください"
    exit 1
fi

# venv_sim に足りないパッケージを /tmp venv から補完
if [ -f "/tmp/robot_dslc8_venv/bin/activate" ]; then
    source /tmp/robot_dslc8_venv/bin/activate
fi

# gear_sonic を Python パスに追加してシム起動
cd "$EXTERN"
PYTHONPATH="$EXTERN:${PYTHONPATH:-}" "$PYTHON" gear_sonic/scripts/run_sim_loop.py \
    --enable-offscreen \
    --enable-image-publish \
    "$@"
