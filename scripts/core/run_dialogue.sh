#!/bin/bash
REPO_ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
CONFIG="$REPO_ROOT/configs/config.local.yaml"

if [ ! -f "$CONFIG" ]; then
    echo "エラー: $CONFIG が見つかりません"
    echo "configs/config.yaml をコピーして config.local.yaml を作成してください"
    exit 1
fi

export OPENAI_API_KEY=$(python3 -c "
import yaml
with open('$CONFIG') as f:
    c = yaml.safe_load(f)
print(c.get('openai', {}).get('api_key', ''))
")

export OPENAI_REALTIME_MODEL=$(python3 -c "
import yaml
with open('$CONFIG') as f:
    c = yaml.safe_load(f)
print(c.get('openai', {}).get('realtime_model', 'gpt-4o-realtime-preview'))
")

if [ -z "$OPENAI_API_KEY" ] || [ "$OPENAI_API_KEY" = "your-api-key-here" ]; then
    echo "エラー: OPENAI_API_KEY が設定されていません"
    echo "$CONFIG に API キーを設定してください"
    exit 1
fi

# Python 環境を決定: g1_deploy conda > /tmp/robot_dslc8_venv > system python3
if command -v conda &>/dev/null && conda activate g1_deploy 2>/dev/null; then
    PYTHON=python
elif [ -f "/tmp/robot_dslc8_venv/bin/python" ]; then
    source /tmp/robot_dslc8_venv/bin/activate
    PYTHON=python
else
    PYTHON=python3
fi

cd "$REPO_ROOT/src/dialogue_system"

$PYTHON g1_realtime_dialogue.py "$@"
