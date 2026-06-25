#!/bin/bash
# setup_environment.sh
# 実機Ubuntu上で実行するワンショット環境構築スクリプト
# 問題1 (submodule), 問題2 (conda), 問題3 (pyaudio), 問題6 (Docker), 問題7 (HuggingFace) を解決する
#
# 使い方:
#   cd <このリポジトリのルート>
#   bash scripts/setup_environment.sh
#
# 前提:
#   - Ubuntu 22.04 以降
#   - インターネット接続
#   - sudo 権限

set -e
REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$REPO_ROOT"

RED='\033[0;31m'; YELLOW='\033[1;33m'; GREEN='\033[0;32m'; NC='\033[0m'
info()  { echo -e "${GREEN}[INFO]${NC}  $*"; }
warn()  { echo -e "${YELLOW}[WARN]${NC}  $*"; }
error() { echo -e "${RED}[ERROR]${NC} $*"; }

# ============================================================
# 問題1: git submodule の初期化
# ============================================================
echo ""
echo "========================================"
echo "  問題1: git submodule を初期化"
echo "========================================"

if [ ! -d ".git" ]; then
  warn ".git ディレクトリが見つかりません。"
  warn "このリポジトリは ZIP 展開などで取得されたため git 管理外です。"
  warn "手動で submodule を clone します。"

  # GR00T-WholeBodyControl
  if [ -z "$(ls -A extern/GR00T-WholeBodyControl 2>/dev/null | grep -v '^gear_sonic')" ]; then
    info "extern/GR00T-WholeBodyControl を clone 中..."
    # パッチ適用済みディレクトリが存在する場合は中身を保持しながら clone
    TMP_GR00T=$(mktemp -d)
    git clone --depth 1 --no-tags \
      https://github.com/NVlabs/GR00T-WholeBodyControl "$TMP_GR00T/gr00t"
    rsync -a --ignore-existing "$TMP_GR00T/gr00t/" extern/GR00T-WholeBodyControl/
    rm -rf "$TMP_GR00T"
    info "GR00T-WholeBodyControl: clone 完了"
  else
    info "GR00T-WholeBodyControl: 既存コンテンツあり、スキップ"
  fi

  # kimodo
  if [ -z "$(ls -A extern/kimodo 2>/dev/null | grep -v 'docker-compose')" ]; then
    info "extern/kimodo を clone 中..."
    TMP_KIMODO=$(mktemp -d)
    git clone --depth 1 --no-tags \
      https://github.com/nv-tlabs/kimodo "$TMP_KIMODO/kimodo"
    rsync -a --exclude='docker-compose.yaml' --ignore-existing \
      "$TMP_KIMODO/kimodo/" extern/kimodo/
    rm -rf "$TMP_KIMODO"
    info "kimodo: clone 完了"
  else
    info "kimodo: 既存コンテンツあり、スキップ"
  fi
else
  info "git submodule update --init --recursive を実行中..."
  git submodule update --init --recursive
fi

# git-lfs pull（GR00T のモデルウェイト）
if command -v git-lfs &>/dev/null; then
  info "git lfs pull (GR00T-WholeBodyControl)..."
  (cd extern/GR00T-WholeBodyControl && git lfs pull) || \
    warn "git lfs pull 失敗。手動で実行してください: cd extern/GR00T-WholeBodyControl && git lfs pull"
else
  warn "git-lfs が未インストールです。以下で導入後に再実行してください:"
  warn "  sudo apt-get install -y git-lfs && git lfs install"
  warn "  cd extern/GR00T-WholeBodyControl && git lfs pull"
fi

# kimodo-viser（Docker ビルドに必要、submodule 外）
if [ ! -d "extern/kimodo/kimodo-viser" ]; then
  info "extern/kimodo/kimodo-viser を clone 中..."
  git clone --depth 1 https://github.com/nv-tlabs/kimodo-viser \
    extern/kimodo/kimodo-viser
  info "kimodo-viser: clone 完了"
else
  info "kimodo-viser: 既存、スキップ"
fi

# パッチ適用
info "パッチ適用中..."
bash scripts/patches/apply_patches.sh

# ============================================================
# 問題3: pyaudio ビルド依存 (libportaudio) の解決
# ============================================================
echo ""
echo "========================================"
echo "  問題3: pyaudio ビルド依存を解決"
echo "========================================"

info "libportaudio2 / portaudio19-dev をインストール..."
sudo apt-get update -qq
sudo apt-get install -y libportaudio2 portaudio19-dev
info "portaudio: インストール完了"

# ============================================================
# 問題6: Docker のインストール確認
# ============================================================
echo ""
echo "========================================"
echo "  問題6: Docker のセットアップ"
echo "========================================"

if ! command -v docker &>/dev/null; then
  info "Docker をインストール中..."
  curl -fsSL https://get.docker.com | sudo sh
  sudo usermod -aG docker "$USER"
  info "Docker: インストール完了"
  warn "docker グループ反映のため、一度ログアウト/再ログインするか 'newgrp docker' を実行してください。"
else
  info "Docker: 既インストール ($(docker --version))"
fi

# Docker ログイン確認（kimodo text-encoder build 用）
info "Docker が動作可能か確認..."
docker info &>/dev/null || warn "Docker デーモンが起動していません。'sudo systemctl start docker' を実行してください。"

# ============================================================
# 問題2: conda / g1_deploy 環境の構築
# ============================================================
echo ""
echo "========================================"
echo "  問題2: conda g1_deploy 環境を構築"
echo "========================================"

# conda が未インストールの場合は Miniconda を導入
if ! command -v conda &>/dev/null; then
  info "Miniconda をインストール中..."
  MINICONDA_SH=$(mktemp /tmp/miniconda_XXXXXX.sh)
  curl -fsSL "https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh" \
    -o "$MINICONDA_SH"
  bash "$MINICONDA_SH" -b -p "$HOME/miniconda3"
  rm "$MINICONDA_SH"
  # PATH に追加
  export PATH="$HOME/miniconda3/bin:$PATH"
  conda init bash
  info "Miniconda: インストール完了"
  warn "conda を有効化するため、新しいシェルを開くか 'source ~/.bashrc' を実行してください。"
fi

# conda コマンドを有効化
# shellcheck disable=SC1090
[ -f "$HOME/miniconda3/etc/profile.d/conda.sh" ] && \
  source "$HOME/miniconda3/etc/profile.d/conda.sh"
[ -f "/opt/conda/etc/profile.d/conda.sh" ] && \
  source "/opt/conda/etc/profile.d/conda.sh"

if command -v conda &>/dev/null; then
  # g1_deploy 環境の作成
  if conda env list | grep -q "^g1_deploy "; then
    info "conda 環境 g1_deploy: 既存、パッケージを更新"
  else
    info "conda 環境 g1_deploy (Python 3.13) を作成中..."
    conda create -y -n g1_deploy python=3.13
    info "g1_deploy: 作成完了"
  fi

  info "requirements_deploy.txt をインストール中..."
  conda run -n g1_deploy pip install -r environments/requirements_deploy.txt
  conda run -n g1_deploy pip install pyaudio
  info "g1_deploy: パッケージインストール完了"
else
  warn "conda が使えません。手動で以下を実行してください:"
  warn "  source ~/.bashrc"
  warn "  conda create -y -n g1_deploy python=3.13"
  warn "  conda activate g1_deploy"
  warn "  pip install -r environments/requirements_deploy.txt pyaudio"
fi

# venv_sim の requirements インストール
if [ -f "extern/GR00T-WholeBodyControl/.venv_sim/bin/activate" ]; then
  info "venv_sim の requirements_sim.txt をインストール中..."
  source extern/GR00T-WholeBodyControl/.venv_sim/bin/activate
  pip install -r environments/requirements_sim.txt
  deactivate
  info "venv_sim: パッケージインストール完了"
else
  warn ".venv_sim が存在しません。以下を先に実行してください:"
  warn "  bash extern/GR00T-WholeBodyControl/install_scripts/install_mujoco_sim.sh"
  warn "その後、再度このスクリプトを実行するか以下を手動実行:"
  warn "  source extern/GR00T-WholeBodyControl/.venv_sim/bin/activate"
  warn "  pip install -r environments/requirements_sim.txt"
fi

# ============================================================
# 問題7: HuggingFace Llama 3 トークン設定
# ============================================================
echo ""
echo "========================================"
echo "  問題7: HuggingFace トークンの確認"
echo "========================================"

if [ -z "$HF_TOKEN" ] && ! huggingface-cli whoami &>/dev/null 2>&1; then
  warn "HuggingFace にログインしていません。"
  warn "Kimodo の text encoder (Meta Llama 3) を使うには以下の手順が必要です:"
  warn ""
  warn "  手順 A: ライセンス同意"
  warn "    ブラウザで https://huggingface.co/meta-llama/Meta-Llama-3-8B を開き"
  warn "    「Access repository」でライセンスに同意してください。"
  warn ""
  warn "  手順 B: トークンの取得と設定"
  warn "    https://huggingface.co/settings/tokens で Read トークンを発行し:"
  warn "      export HF_TOKEN=hf_xxxx        # 一時的に設定"
  warn "      echo 'export HF_TOKEN=hf_xxxx' >> ~/.bashrc  # 永続化"
  warn "    または:"
  warn "      pip install huggingface_hub && huggingface-cli login"
  warn ""
  warn "  トークンを設定してから Kimodo Docker イメージをビルドしてください:"
  warn "    cd extern/kimodo && docker compose build text-encoder"
else
  info "HuggingFace: ログイン済み"
fi

# ============================================================
# 完了サマリー
# ============================================================
echo ""
echo "========================================"
echo "  セットアップ完了"
echo "========================================"
echo ""
echo "残りの手動作業:"
echo "  1. TensorRT (約10GB) を NVIDIA Developer からダウンロード:"
echo "       https://developer.nvidia.com/tensorrt"
echo "       TAR パッケージを展開し TensorRT_ROOT を設定"
echo "       sudo apt-get install -y pv"
echo "       pv TensorRT-*.tar.gz | tar -xz -f -"
echo "       mv TensorRT-* ~/TensorRT"
echo "       echo 'export TensorRT_ROOT=\$HOME/TensorRT' >> ~/.bashrc"
echo ""
echo "  2. GR00T Deploy 環境のビルド (TensorRT 設定後):"
echo "       cd extern/GR00T-WholeBodyControl/gear_sonic_deploy"
echo "       ./scripts/install_deps.sh && source scripts/setup_env.sh && just build"
echo ""
echo "  3. Kimodo Docker イメージのビルド (HF_TOKEN 設定後):"
echo "       cd extern/kimodo && docker compose build text-encoder"
echo ""
echo "  4. モーションデータ生成 (シミュレーター起動後):"
echo "       bash scripts/motion/generate_motions.sh"
