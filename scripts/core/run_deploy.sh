#!/bin/bash
REPO_ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
EXTERN="$REPO_ROOT/extern/GR00T-WholeBodyControl/gear_sonic_deploy"

cd "$EXTERN"

# GPU設定
GPU_SETTINGS="--gpus all"
# TensorRT ホストパスを自動検出（環境変数 TENSORRT_ROOT > /opt/tensorrt > /home/unitree-g1/TensorRT）
if [ -n "${TENSORRT_ROOT:-}" ] && [ -d "$TENSORRT_ROOT" ]; then
    _TENSORRT_HOST="$TENSORRT_ROOT"
elif [ -d "/opt/tensorrt" ]; then
    _TENSORRT_HOST="/opt/tensorrt"
elif [ -d "/home/unitree-g1/TensorRT" ] && [ -n "$(ls -A /home/unitree-g1/TensorRT 2>/dev/null)" ]; then
    _TENSORRT_HOST="/home/unitree-g1/TensorRT"
else
    echo "[WARNING] TensorRT directory not found. Build may fail." >&2
    _TENSORRT_HOST="/home/unitree-g1/TensorRT"
fi
echo "[run_deploy] TensorRT host path: $_TENSORRT_HOST"
TENSORRT_MOUNT="-v ${_TENSORRT_HOST}:/opt/TensorRT:ro"
DIALOGUE_HOST="${DIALOGUE_HOST:-localhost}"
DIALOGUE_ZMQ_PORT="${DIALOGUE_ZMQ_PORT:-5556}"

# 既存コンテナを停止
docker stop g1-deploy-dev 2>/dev/null || true
sleep 1

# コンテナが既に起動中か確認
if ! docker ps --format '{{.Names}}' | grep -q "^g1-deploy-dev$"; then
    echo "Docker コンテナを起動中..."
    docker run -d --rm \
        --name g1-deploy-dev \
        --network host \
        $GPU_SETTINGS \
        -v "$(pwd):/workspace/g1_deploy:rw" \
        $TENSORRT_MOUNT \
        -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
        -e ROS_DOMAIN_ID=0 \
        -e NVIDIA_VISIBLE_DEVICES=all \
        -e NVIDIA_DRIVER_CAPABILITIES=compute,utility \
        -w /workspace/g1_deploy \
        g1-deploy-dev \
        sleep infinity
    echo "コンテナ起動待機中..."
    sleep 3
fi

echo "deploy.sh を実行中..."
echo "[ZMQ] Dialogue host: ${DIALOGUE_HOST}:${DIALOGUE_ZMQ_PORT}"
docker exec -it -e DEPLOY_YES=1 g1-deploy-dev bash -lc "source scripts/setup_env.sh && bash deploy.sh --input-type zmq_manager --zmq-host '${DIALOGUE_HOST}' --zmq-port '${DIALOGUE_ZMQ_PORT}' sim"
