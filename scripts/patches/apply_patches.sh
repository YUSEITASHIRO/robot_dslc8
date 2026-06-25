#!/bin/bash
# apply_patches.sh
# patches/ のファイルを extern/ にシンボリックリンクで適用
# extern/ の git 履歴は変更しない

set -e

REPO_ROOT="$(cd "$(dirname "$0")/../.." && pwd)"

# パッチディレクトリ → extern ディレクトリ のマッピング
declare -A PATCH_MAP=(
    ["$REPO_ROOT/patches/gr00t"]="$REPO_ROOT/extern/GR00T-WholeBodyControl"
    ["$REPO_ROOT/patches/kimodo"]="$REPO_ROOT/extern/kimodo"
)

apply_patch_dir() {
    local PATCH="$1"
    local EXTERN="$2"

    if [ ! -d "$EXTERN" ]; then
        echo "エラー: $EXTERN が見つかりません"
        echo "以下を実行してください:"
        echo "  git submodule update --init --recursive"
        exit 1
    fi

    echo "PATCH : $PATCH"
    echo "EXTERN: $EXTERN"

    find "$PATCH" -type f | grep -v '__pycache__' | while read patch_file; do
        rel="${patch_file#$PATCH/}"
        target="$EXTERN/$rel"
        target_dir="$(dirname "$target")"

        mkdir -p "$target_dir"

        # 元ファイルが存在する場合はバックアップ（.orig）— 初回のみ保存
        if [ -f "$target" ] && [ ! -L "$target" ] && [ ! -f "${target}.orig" ]; then
            cp "$target" "${target}.orig"
            echo "[backup] $rel"
        fi

        # XML / sh ファイルはコピー（Docker 内から symlink が解決できないため）、その他はシンボリックリンク
        if [[ "$rel" == *.xml || "$rel" == *.sh ]]; then
            cp --remove-destination "$patch_file" "$target"
            echo "[copy]   $rel"
        else
            ln -sf "$patch_file" "$target"
            echo "[link]   $rel"
        fi
    done
}

echo "=== パッチ適用開始 (シンボリックリンク方式) ==="
echo ""

for patch_dir in "${!PATCH_MAP[@]}"; do
    if [ -d "$patch_dir" ]; then
        apply_patch_dir "$patch_dir" "${PATCH_MAP[$patch_dir]}"
        echo ""
    fi
done

echo "=== 完了 ==="
echo "元ファイルは .orig として保存されています"
echo "元に戻す場合: bash scripts/remove_patches.sh"
