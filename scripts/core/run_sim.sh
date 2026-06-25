#!/bin/bash
# MuJoCo シミュレーターを起動するラッパー
#
# 使い方:
#   bash scripts/core/run_sim.sh              # 通常モード（カメラ離屏レンダリング有効）
#   CAMERA=0 bash scripts/core/run_sim.sh     # 純制御モード（離屏レンダリング無効）
#   bash scripts/core/run_sim.sh --no-camera  # 同上（引数でも指定可）
#
# 純制御モードは、メイン制御ループから同期的な離屏レンダリング(renderer.render())を
# 外し、gear-sonic との 200Hz 制御通信のリアルタイム性だけを検証するためのもの。
# WSL でロボットが不安定になる場合、CAMERA=0 で安定するなら原因は離屏レンダリング。

REPO_ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
EXTERN="$REPO_ROOT/extern/GR00T-WholeBodyControl"

# WSL: 強制的に GPU(d3d12) でレンダリング。未設定だと llvmpipe(CPU) に落ちて激しく重くなる。
# /dev/dxg があれば WSL とみなして GPU パスを有効化する。
if [ -e /dev/dxg ]; then
    export GALLIUM_DRIVER=d3d12
    export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
fi

# カメラ（離屏レンダリング）の ON/OFF。
# 環境変数 CAMERA=0 もしくは引数 --no-camera で無効化できる。
CAMERA="${CAMERA:-1}"
PASSTHRU=()
for arg in "$@"; do
    case "$arg" in
        --no-camera) CAMERA=0 ;;
        --camera)    CAMERA=1 ;;
        *)           PASSTHRU+=("$arg") ;;
    esac
done

CAMERA_FLAGS=()
if [ "$CAMERA" != "0" ]; then
    CAMERA_FLAGS=(--enable-offscreen --enable-image-publish)
    echo "[run_sim] camera mode: ON  (offscreen render enabled)"
else
    echo "[run_sim] camera mode: OFF (control-only, offscreen render disabled)"
fi

source "$EXTERN/.venv_sim/bin/activate"
cd "$EXTERN"

python gear_sonic/scripts/run_sim_loop.py \
    "${CAMERA_FLAGS[@]}" \
    "${PASSTHRU[@]}"
