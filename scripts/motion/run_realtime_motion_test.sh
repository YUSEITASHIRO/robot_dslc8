#!/bin/bash
# run_realtime_motion_test.sh — テキスト入力 → リアルタイムモーション生成・再生
#
# 事前準備:
#   Terminal 1: bash scripts/run_sim.sh
#   Terminal 2: bash scripts/run_deploy.sh
#   Terminal 3: このスクリプト  ← text encoder Docker を自動起動
#
# Usage:
#   bash scripts/run_realtime_motion_test.sh
#   bash scripts/run_realtime_motion_test.sh --dur 5 --steps 100

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
KIMODO_DIR="$REPO_ROOT/extern/kimodo"

PYTHON="${ROBOT_DSLC8_PYTHON:-}"
if [ -z "$PYTHON" ]; then
    if [ -f "/tmp/robot_dslc8_venv/bin/python" ]; then
        PYTHON=/tmp/robot_dslc8_venv/bin/python
    else
        PYTHON=python3
    fi
fi
TE_PORT=9550
TE_URL="http://127.0.0.1:${TE_PORT}/"
DOCKER_STARTED=0

te_running() {
  curl -sf "${TE_URL}" -o /dev/null 2>/dev/null
}

# ── text encoder Docker を確認・起動 ─────────────────────────
if te_running; then
  echo "[TextEncoder] 既に起動済み (port ${TE_PORT})"
else
  echo "[TextEncoder] Docker コンテナを起動中..."
  cd "$KIMODO_DIR"
  docker compose up -d text-encoder
  DOCKER_STARTED=1

  echo -n "[TextEncoder] 起動待ち..."
  WAIT=0
  until te_running || [ $WAIT -ge 300 ]; do
    sleep 5; WAIT=$((WAIT + 5)); echo -n "."
  done
  echo ""

  if ! te_running; then
    echo "[ERROR] text encoder の起動に失敗しました"
    docker compose logs text-encoder | tail -20
    exit 1
  fi
  echo "[TextEncoder] 起動完了 (${WAIT}s)"
fi

# ── 終了時に Docker を停止（このスクリプトが起動した場合のみ）─
cleanup() {
  if [ "$DOCKER_STARTED" = "1" ]; then
    echo ""
    echo "[TextEncoder] Docker コンテナを停止中..."
    cd "$KIMODO_DIR" && docker compose stop text-encoder
  fi
}
trap cleanup EXIT INT TERM

# ── 起動 ──────────────────────────────────────────────────────
conda activate g1_deploy 2>/dev/null || true
cd "$REPO_ROOT"

echo ""
exec "$PYTHON" "$REPO_ROOT/src/motion/g1_realtime_motion_test.py" "$@"
