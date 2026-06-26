#!/bin/bash
# WebUI (bridge.py) への接続スクリプト
# 実行すると SSH トンネル確立 + g24 で bridge.py 起動 + ブラウザを開く

set -e

REMOTE="g24"
REPO="/home/tashiro/robot_dslc8"
BRIDGE_PORT=8765
SOCKET="/tmp/ssh_ctl_g24"

echo "[WebUI] SSH トンネルを確立中..."

# 既存の ControlMaster があれば一度閉じる
ssh -S "$SOCKET" -O exit "$REMOTE" 2>/dev/null || true

# ControlMaster + ポートフォワード (-L) でバックグラウンド接続
ssh -fNM \
    -S "$SOCKET" \
    -L "${BRIDGE_PORT}:localhost:${BRIDGE_PORT}" \
    "$REMOTE"

echo "[WebUI] トンネル確立: localhost:${BRIDGE_PORT} → ${REMOTE}:${BRIDGE_PORT}"

# g24 上で bridge.py を tmux セッション robot:3 で起動（なければ作る）
# ControlMaster 経由は X11 制約があるため直接接続で実行
ssh "$REMOTE" bash <<'REMOTE_SCRIPT'
REPO="/home/tashiro/robot_dslc8"
SESSION="robot"
WINDOW=3

# tmux セッションがなければ作成
if ! tmux has-session -t "${SESSION}" 2>/dev/null; then
    tmux new-session -d -s "${SESSION}" -n "sim"
    for i in 1 2 3; do
        tmux new-window -t "${SESSION}:${i}"
    done
fi

# bridge.py がすでに動いているか確認
if tmux list-panes -t "${SESSION}:${WINDOW}" -F "#{pane_current_command}" 2>/dev/null | grep -q "python"; then
    echo "[WebUI] bridge.py はすでに起動中です"
    exit 0
fi

# tmux window 3 で bridge.py を起動
tmux send-keys -t "${SESSION}:${WINDOW}" "" C-c 2>/dev/null || true
sleep 0.3
tmux send-keys -t "${SESSION}:${WINDOW}" \
    "cd ${REPO}/webui && source /tmp/robot_dslc8_venv/bin/activate 2>/dev/null; python bridge.py" Enter
echo "[WebUI] bridge.py を起動しました (robot:3)"
REMOTE_SCRIPT

echo "[WebUI] 起動待機中..."
sleep 2

# ブラウザを開く (Windows / macOS / Linux 対応)
URL="http://localhost:${BRIDGE_PORT}"
if command -v start &>/dev/null; then
    start "$URL"
elif command -v open &>/dev/null; then
    open "$URL"
elif command -v xdg-open &>/dev/null; then
    xdg-open "$URL"
fi

echo "[WebUI] 接続完了: $URL"
echo "[WebUI] 切断するには: ssh -S $SOCKET -O exit $REMOTE"
