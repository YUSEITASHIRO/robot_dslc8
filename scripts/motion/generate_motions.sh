#!/usr/bin/env bash
# generate_motions.sh — Kimodo G1 動作バッチ生成
#
# text encoder Docker を自動起動し、生成完了後に停止します。
# 既に起動済みの場合はそのまま使用し、終了時も停止しません。
#
# Usage:
#   bash scripts/generate_motions.sh                     # 全動作
#   bash scripts/generate_motions.sh --motions nod wave  # 指定動作のみ
#   bash scripts/generate_motions.sh --samples 1 --steps 50  # 速度優先
#   bash scripts/generate_motions.sh --list              # 動作一覧

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
KIMODO_DIR="$REPO_ROOT/extern/kimodo"

PYTHON=/home/unitree-g1/anaconda3/bin/python
TE_PORT=9550
TE_URL="http://127.0.0.1:${TE_PORT}/"
DOCKER_STARTED=0

echo "=== Kimodo G1 動作生成 ==="
echo "出力先: $REPO_ROOT/data/motions/"
echo ""

# ── リスト表示は即終了 ────────────────────────────────────────
for arg in "$@"; do
  if [ "$arg" = "--list" ]; then
    exec "$PYTHON" "$REPO_ROOT/src/motion/generate_motions.py" --list
  fi
done

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

# ── 動作生成 ──────────────────────────────────────────────────
echo ""
exec "$PYTHON" "$REPO_ROOT/src/motion/generate_motions.py" "$@"
