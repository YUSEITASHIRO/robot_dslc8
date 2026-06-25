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

conda activate g1_deploy 2>/dev/null || true
cd "$REPO_ROOT/src/dialogue_system"

python g1_realtime_dialogue.py "$@"
