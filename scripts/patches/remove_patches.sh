#!/bin/bash
# remove_patches.sh
# シンボリックリンクを削除して .orig を元に戻す

REPO_ROOT="$(cd "$(dirname "$0")/../.." && pwd)"

# パッチディレクトリ → extern ディレクトリ のマッピング
declare -A PATCH_MAP=(
    ["$REPO_ROOT/patches/gr00t"]="$REPO_ROOT/extern/GR00T-WholeBodyControl"
    ["$REPO_ROOT/patches/kimodo"]="$REPO_ROOT/extern/kimodo"
)

remove_patch_dir() {
    local PATCH="$1"
    local EXTERN="$2"

    find "$PATCH" -type f | grep -v '__pycache__' | while read patch_file; do
        rel="${patch_file#$PATCH/}"
        target="$EXTERN/$rel"

        if [ -L "$target" ] || [ -f "$target" ]; then
            if [ -f "${target}.orig" ]; then
                rm "$target"
                mv "${target}.orig" "$target"
                echo "[restored] $rel"
            elif [ -L "$target" ]; then
                rm "$target"
                echo "[removed]  $rel (バックアップなし)"
            else
                # .orig が無い通常ファイルは git checkout でリストア
                git -C "$EXTERN" checkout -- "$rel" 2>/dev/null && \
                    echo "[git-restore] $rel" || \
                    echo "[skip]  $rel (バックアップなし・git checkout 不可)"
            fi
        fi
    done
}

echo "=== パッチ削除開始 ==="

for patch_dir in "${!PATCH_MAP[@]}"; do
    if [ -d "$patch_dir" ]; then
        remove_patch_dir "$patch_dir" "${PATCH_MAP[$patch_dir]}"
    fi
done

echo "=== 完了 ==="
