# mobile-robot-dialogue-system

Unitree G1 ロボット向けリアルタイム音声対話システム。  
OpenAI Realtime API による音声対話 + Kimodo によるモーション生成 + GR00T WBC によるロボット制御を統合したプラットフォームです。

参加者はこのリポジトリをベースに、モーション・移動・対話の各機能を自由に拡張・改良して競技に臨んでください。

---

## ディレクトリ構成

```
extern/
  GR00T-WholeBodyControl/   # GR00T WBC (deploy + MuJoCo sim)
  kimodo/                   # Kimodo モーション生成モデル
patches/                    # extern への変更パッチ（シンボリックリンク方式）
  gr00t/                    # GR00T-WholeBodyControl へのパッチ
  kimodo/                   # kimodo へのパッチ
src/
  dialogue_system/          # メイン音声対話システム
  motion/                   # モーション生成・テストツール
  move/                     # 移動・ナビゲーションツール
scripts/
  core/                     # 基本起動スクリプト (sim / deploy / dialogue)
  motion/                   # モーション関連スクリプト
  move/                     # 移動関連スクリプト
  patches/                  # パッチ適用・削除スクリプト
data/
  motions/                  # 生成済みモーションデータ (.npz)
configs/                    # 設定ファイル
```

---

## セットアップ

### Step 1: リポジトリのクローン

```bash
git clone https://github.com/YUSEITASHIRO/robot_dslc8.git
cd robot_dslc8
git submodule update --init --recursive
cd extern/GR00T-WholeBodyControl
git lfs pull
```

Kimodo の Docker ビルドに必要な `kimodo-viser` は submodule ではないため、別途 clone してください：

```bash
cd extern/kimodo
git clone https://github.com/nv-tlabs/kimodo-viser.git
```

> **注意:** `extern/kimodo` は git submodule です。`apply_patches.sh` を実行すると `docker-compose.yaml` がシンボリックリンクに変わるため、`extern/kimodo` 内で `git status` を実行すると typechange (`T`) として表示されます。また `docker-compose.yaml.orig`（バックアップ）と `kimodo-viser/`（手動 clone）が untracked として表示されます。これらは正常な状態です。**`extern/kimodo` に直接コミットしないでください。**

その後、Kimodo の Docker イメージをビルドします（初回のみ・数分かかります）：

> **⚠️ 事前に必要: Meta Llama 3 ライセンス署名**  
> Kimodo の text encoder は内部で **Meta Llama 3 (8B)** を使用しています。  
> HuggingFace からの自動ダウンロードには、事前に以下のページでライセンス同意が必要です。  
> 未署名のままビルド・実行すると LoRA weights の読み込みエラーが発生します。  
> → https://huggingface.co/meta-llama/Meta-Llama-3-8B

```bash
cd extern/kimodo
docker compose build text-encoder
```

---

### Step 2: Deploy 環境のセットアップ（GR00T WBC）

公式ドキュメント: https://nvlabs.github.io/GR00T-WholeBodyControl/getting_started/installation_deploy.html

#### A: TensorRT のインストール

> ⚠️ **重要:** 必ず指定バージョンの TensorRT を使用してください。異なるバージョンを使用するとプランナーが誤った動作を出力し、**ロボットの危険な動作を引き起こす可能性があります。**

NVIDIA Developer から TAR パッケージ（DEB ではなく）をダウンロードしてください。アーカイブは約 10GB です。

```bash
sudo apt-get install -y pv
pv TensorRT-*.tar.gz | tar -xz -f -
mv TensorRT-* ~/TensorRT
echo 'export TensorRT_ROOT=$HOME/TensorRT' >> ~/.bashrc
source ~/.bashrc
```

#### B: Native 環境の構築

```bash
cd extern/GR00T-WholeBodyControl/gear_sonic_deploy

chmod +x scripts/install_deps.sh
./scripts/install_deps.sh

source scripts/setup_env.sh
echo "source $(pwd)/scripts/setup_env.sh" >> ~/.bashrc

just build
```

#### C: Docker (ROS2 開発環境)

```bash
pip install huggingface_hub
cd extern/GR00T-WholeBodyControl
python download_from_hf.py
```

```bash
sudo usermod -aG docker $USER
newgrp docker

export TensorRT_ROOT=/path/to/TensorRT

cd extern/GR00T-WholeBodyControl/gear_sonic_deploy
./docker/run-ros2-dev.sh

# コンテナ内で
source scripts/setup_env.sh
just build
```

#### D: Simulator 環境のセットアップ（MuJoCo Sim）

```bash
cd extern/GR00T-WholeBodyControl
bash install_scripts/install_mujoco_sim.sh
```

これで `.venv_sim` が作成されます。

---

### Step 3: パッチの適用

GR00T・Kimodo への変更をシンボリックリンクで適用します。

```bash
bash scripts/patches/apply_patches.sh
```

> **注意:** `git submodule update` 実行後は必ず再度実行してください。

---

### Step 4: 設定ファイルの作成

```bash
cp configs/config.yaml configs/config.local.yaml
```

`configs/config.local.yaml` を編集して OpenAI API キーを設定してください：

```yaml
openai:
  api_key: "sk-..."
```

> `config.local.yaml` は `.gitignore` により git に上がりません。

---

## 起動

**ターミナル 1 → 2 の順に起動し、ターミナル 2 の `Init Done` を確認してからターミナル 3 以降に進んでください。**

### ターミナル 1: MuJoCo シミュレーター

```bash
bash scripts/core/run_sim.sh
```

MuJoCo ウィンドウが開き、ロボットが宙吊り状態で表示されます。  
この時点ではロボットは動きません。

MuJoCo ウィンドウのキー操作：

| キー | 動作 |
|------|------|
| `9` | 吊り上げ状態の G1 を床に下ろす |
| `Back` | 初期状態に戻す |
| `C` | カメラ切替（自由視点 → ego_view → user_eye → counter） |
| `P` | camera viewer ウィンドウ 開閉 |
| `]` | viewer 映像切替 (ego_view ↔ user_eye) |
| `Space` | user model の向きを 180° 切替 |
| 矢印キー | user model を移動 |
| `Home` | user model を Robot 前方にリセット |

### ターミナル 2: Deploy (WBC)

```bash
bash scripts/core/run_deploy.sh
```

スクリプトが自動で `Y` を入力して deploy を開始します。  

**`Init Done` が表示されるまで待ってください。**  
このとき MuJoCo ウィンドウ内のロボットが、脱力した初期姿勢から**直立の ready 姿勢**に切り替わります。  
この変化が確認できて初めて、次のターミナルに進めます。

> Deploy を終了するには `o` キーを押してください。

---

以下はターミナル 2 の `Init Done` 確認後に実行してください。

### (初回のみ) モーションデータの生成

音声対話システムは起動時に `data/motions/` を読み込みます。  
初回は以下を実行してモーションデータを生成してください：

```bash
bash scripts/motion/generate_motions.sh
```

生成には Kimodo の Docker text encoder が必要です（自動起動されます）。

### ターミナル 3: 音声対話システム

```bash
bash scripts/core/run_dialogue.sh
```

起動後は音声対話が始まります。キーボードでも手動操作できます：

| キー | 動作 |
|------|------|
| `W` | 前進 |
| `S` | 後退 |
| `A` | 左旋回 |
| `D` | 右旋回 |
| `Space` | 停止 |

> **参加者へ:** 現状の `run_dialogue.sh` は音声対話と事前生成モーションの再生のみを含みます。**モーション生成・移動ナビゲーションとの統合は参加者自身が実装してください。** 各機能の詳細は後述の「[モーション機能](#モーション機能kimodo-ベース)」「[移動機能](#移動機能グリッド座標ナビゲーション)」を参照してください。

### 停止方法

**停止順序は起動と逆順にしてください。**

#### 手順 1: ターミナル 3 以降を停止（dialogue / motion / move）
実行中のターミナルで `Ctrl + C`

#### 手順 2: ターミナル 2 を停止（Deploy）
run_deploy.sh のターミナルで **`o`** キーを押す  
→ `[ZMQManager] EMERGENCY STOP` と表示され、自動的にシャットダウンしてシェルに戻ります

#### 手順 3: ターミナル 1 を停止（MuJoCo シミュレーター）
run_sim.sh のターミナルで `Ctrl + C`

---

## パッチ管理

`extern/` への変更はすべて `patches/` 経由で管理します。**`extern/` 以下に直接コミットしないでください。**

```bash
bash scripts/patches/apply_patches.sh   # 適用
bash scripts/patches/remove_patches.sh  # 削除（元に戻す）
```

| ファイル種別 | 適用方法 | 編集後の反映 |
|------------|---------|------------|
| `.py` / `.yaml` | シンボリックリンク | 即時反映（同一ファイル） |
| `.xml` / `.sh` | コピー | `apply_patches.sh` を再実行 |

`git submodule update` 実行後は必ず `apply_patches.sh` を再実行してください。

#### extern/kimodo の git 状態について

`apply_patches.sh` 実行後、`extern/kimodo` で `git status` を確認すると以下が表示されますが、いずれも正常です：

```
 T docker-compose.yaml          ← symlink に変わったため typechange 表示（正常）
?? docker-compose.yaml.orig     ← apply_patches.sh が作成したバックアップ
?? kimodo-viser/                ← 手動 clone したディレクトリ
```

これらは無視して問題ありません。`extern/kimodo` 内でコミットや `git checkout` は行わないでください。

---

## 環境一覧

| 環境 | 用途 | 場所 |
|------|------|------|
| `.venv_sim` | MuJoCo シミュレーター | `extern/GR00T-WholeBodyControl/.venv_sim` |
| `g1_deploy` | Deploy + Dialogue | bashrc 環境 |

---

## src/ — 実装・開発ガイド

参加者が主に触れるコードは `src/` 以下にあります。

```
src/
  dialogue_system/
    g1_realtime_dialogue.py     # メイン音声対話システム（エントリポイント）
    mujoco_viewer.py            # MuJoCo カメラ映像をウィンドウ表示するビューア
  motion/
    generate_motions.py         # Kimodo によるモーションバッチ生成 → .npz 保存
    g1_motion_test.py           # 生成済み .npz モーションの再生テスト CLI
    g1_realtime_motion_test.py  # テキスト入力 → Kimodo 生成 → 即時再生 (対話型)
  move/
    g1_move_test.py             # グリッド座標ナビゲーション CLI
    g1_simple_walk.py           # WASD キーボード手動操作
```

### 現状の機能分離について

`src/motion/` と `src/move/` はそれぞれ**単独テスト用スクリプト**です。  
現時点では `g1_realtime_dialogue.py` とは独立しており、同時実行はできません。

| スクリプト | 用途 | 統合状況 |
|-----------|------|---------|
| `g1_realtime_dialogue.py` | 音声対話 + 事前生成モーション再生 | **メイン実装** |
| `g1_motion_test.py` | 生成済みモーションのキーボード手動確認 | 単独テスト用 |
| `g1_realtime_motion_test.py` | プロンプト → リアルタイム生成 → 再生 | 単独テスト用 |
| `g1_move_test.py` | グリッドナビゲーション手動確認 | 単独テスト用 |
| `g1_simple_walk.py` | WASD 手動歩行 | 単独テスト用 |

**参加者への課題:** テスト用スクリプトで動作確認した後、それぞれの機能を `g1_realtime_dialogue.py` に統合することで、音声対話・モーション生成・ナビゲーションを組み合わせたシステムを実現してください。統合の方針は自由です。

---

## モーション機能（Kimodo ベース）

本リポジトリのモーション機能はすべて **[Kimodo](https://github.com/nv-tlabs/kimodo)** による生成がベースです。  
Kimodo はテキストプロンプトから G1 ロボットの関節角度列を生成する拡散モデルです。

### 生成済みモーションデータ

`data/motions/` に各モーション名のディレクトリが存在し、それぞれ `sample_1.npz` を含みます。

```
data/motions/
  wave/sample_1.npz
  bow_45/sample_1.npz
  nod/sample_1.npz
  ... (計 39 種類)
```

各 `.npz` の形式：

```python
jp  # float32[T, 29]  関節角度 (rad) @ 50fps   IsaacLab 順
jv  # float32[T, 29]  関節速度
bq  # float32[T, 4]   体幹クォータニオン (wxyz)
```

### モーションの新規生成

Kimodo を使ったモーション生成には **2 つのアプローチ**があり、どちらを選ぶかは自由です：

| アプローチ | 方法 | 向いているケース |
|-----------|------|----------------|
| **事前生成** | プロンプトで `.npz` を生成して `data/motions/` に保存 | 対話中のレイテンシを減らしたい場合 |
| **リアルタイム生成** | 対話中に Kimodo をオンデマンド呼び出し | 会話内容に応じた柔軟なモーションを作りたい場合 |

#### 事前生成（バッチ）

```bash
bash scripts/motion/generate_motions.sh                       # 全モーション再生成
bash scripts/motion/generate_motions.sh --motions nod wave    # 指定モーションのみ
bash scripts/motion/generate_motions.sh --list                # モーション一覧
```

生成ファイルは `data/motions/<name>/sample_1.npz` に保存され、`run_dialogue.sh` 起動時に自動で読み込まれます。

#### リアルタイム生成（インタラクティブ確認用）

text encoder Docker を自動起動してからモデルを立ち上げます：

```bash
bash scripts/motion/run_realtime_motion_test.sh
```

```
prompt> wave both hands in greeting
prompt> bow forward at 45 degrees
prompt> dur 5       # 生成秒数変更
prompt> steps 100   # 高品質モード (デフォルト: 50)
prompt> play        # 最後の動作を再再生
prompt> q           # 終了
```

対話システムへの組み込みは `src/motion/g1_realtime_motion_test.py` を参考に実装してください。

#### 既存 .npz の再生確認

```bash
bash scripts/motion/run_motion_test.sh
```

### 対話システムとの連携

`g1_realtime_dialogue.py` は起動時に `data/motions/` を動的に読み込み、  
OpenAI の function calling (`select_motion`) でモデルが会話の文脈に合った動作を自律的に選択します。  
モーション名はそのまま動作の説明として機能します（例: `bow_apology`, `wave`, `shrug`）。

### 開発のヒント

- **モーションを追加・差し替える**場合は `generate_motions.py` のプロンプトリストを編集して再生成してください
- **npz を直接作成する**場合は IsaacLab 順 (29 関節)・50fps の形式に合わせてください
- Kimodo の生成品質は `--steps` パラメータに依存します（50=速度優先, 100=高品質）
- 複数サンプルを生成して最良のものを選ぶ運用も有効です

---

## 移動機能（グリッド座標ナビゲーション）

### 座標系

ロボット起動位置を原点とした右手系座標を使用します。

| 軸 | 方向 |
|----|------|
| `+x` | ロボット前方（ユーザー・入口側） |
| `+y` | ロボット左方（店内奥方向） |
| `+z` | 上方 |

### グリッドナビゲーション

店舗内に 1m 間隔のグリッドが設定されており、ロボットはそのセル間を移動します。

```
移動可能セル: x ∈ {-1, 0, 1}、y ∈ {0..6}  計 20 セル
  x = -1 : 右側通路
  x =  0 : 中央通路
  x =  1 : 左側通路
  y =  0 : 入口付近 (Robot 初期位置)
  y =  6 : 店内奥 (y=7,8 はカウンターエリアのため移動不可)
```

```bash
bash scripts/move/run_move_test.sh
```

主なコマンド：

```
go 0 3          # ワールド座標 (x=0, y=3) の最近傍グリッドへ移動
go 1 6          # 左通路の最奥へ移動
go 0 0          # 初期位置へ戻る
go 0 0 n        # 移動後にユーザー側 (+x方向) を向く
status          # 現在位置・向きを確認
walk 3          # 前方に 3 ステップ直進 (グリッド外移動)
face n          # その場で向きを変える (n/s/e/w または度数)
mps 0.32        # METERS_PER_STEP を実測値に合わせてキャリブレーション
h               # ヘルプ表示
```

### WASD 手動操作

```bash
bash scripts/move/run_simple_walk.sh
```

キーを押すたびに 1 ステップずつ移動します（`W`=前進 `S`=後退 `A`=左旋回 `D`=右旋回）。

### 開発のヒント

- 移動は SONIC planner コマンド (ZMQ) で制御しており、物理的な歩行モーションは WBC が担当します
- `walk N` でロボットを N ステップ前進させた後に実測距離を `mps <値>` で更新するとキャリブレーションできます
- グリッドを拡張する場合は `g1_move_test.py` の `_VALID_CELLS` と `scene_43dof.xml` のグリッドマーカーを合わせて編集してください

---

## カメラ映像ビューア

MuJoCo シミュレーターのカメラ映像 (`ego_view`) をウィンドウ表示します。

```bash
bash scripts/core/run_viewer.sh
```

終了は **q** キーまたはウィンドウの × ボタン。

---

## MuJoCo シーン設定

シーン定義は [`patches/gr00t/.../scene_43dof.xml`](patches/gr00t/gear_sonic/data/robot_model/model_data/g1/scene_43dof.xml) で管理しています。  
編集後は必ず `bash scripts/patches/apply_patches.sh` を実行して extern に反映してください。

### 店舗レイアウト

室内サイズ: x∈\[-2, 2\] (幅4m)、y∈\[-1, 9\] (奥行10m)

| 壁 | 位置 | サイズ |
|----|------|--------|
| 奥壁 | y = 9 | 幅4m × 高1.5m |
| 入口壁 | y = −1 | 幅4m × 高1.5m |
| 左壁 | x = 2 (y: 1〜9) | 奥8m × 高1.5m |
| 右壁 | x = −2 (y: −1〜9) | 奥10m × 高1.5m |
| 入口左仕切り | x: 1.0〜2.0, y ≈ 1 | 幅1m × 高1.5m |

### 家具・設備

- **商品陳列棚 (左壁沿い, x = 1.78)**: 奥側 (y: 6〜9) と中央 (y: 1〜6)
- **商品陳列棚 (右壁沿い, x = −1.78)**: 同上
- **追加棚 (入口・右壁沿い)**: x = −1.78, y: −1〜1
- **追加棚 (仕切り前)**: x ≈ 1.5, y ≈ 1.25
- **商談カウンター**: テーブル `(−0.5, 7.8)` + 椅子 3 脚
- **レジカウンター**: 本体 `(0.65, 7.8)` + 端末 `(0.65, 7.45)`

### カメラ

| カメラ名 | 説明 |
|----------|------|
| `(free)` | 自由視点（ドラッグ移動） |
| `ego_view` | Robot 頭部カメラ（fovy=60°） |
| `user_eye` | User body カメラ（fovy=70°） |
| `counter` | 店奥 `(0, 7.5, 2.0)` から入口方向を俯瞰（fovy=80°） |

- Robot 初期位置: `(0, 0, 0.793)`
- User 初期位置: `(1.5, 0, 0.55)`（店内側を向く・eye height ≈ 0.93m = G1 ego_view と同等）

---

## ライセンス

本リポジトリのコード（`src/`・`scripts/`・`patches/`・`configs/`・`data/`）は **MIT License** のもとで提供されます。詳細は [LICENSE](LICENSE) を参照してください。

外部依存ライブラリのライセンスは以下の通りです：

| ライブラリ | ライセンス |
|-----------|-----------|
| [GR00T-WholeBodyControl](https://github.com/NVlabs/GR00T-WholeBodyControl) | Apache 2.0 |
| [Kimodo](https://github.com/nv-tlabs/kimodo) | Apache 2.0 |
