# mobile-robot-dialogue-system

第8回対話システムライブコンペティション（DSLC8）向け、Unitree G1 ロボット用リアルタイム音声対話システム。  
OpenAI Realtime API + Kimodo モーション生成 + GR00T WBC によるロボット制御を統合したプラットフォームです。

---

## コンペティション概要

| 項目 | 内容 |
|------|------|
| **正式名称** | 第8回対話システムライブコンペティション (DSLC8) |
| **テーマ** | ヒューマノイドロボット (Unitree G1) を用いたマルチモーダル対話システム |
| **予選** | MuJoCo Simulator 上でのシミュレーション対話（2026/09/12） |
| **本選** | 実際の G1 ヒューマノイドを用いた対面対話（2026/11/14、早稲田大学） |

---

## シチュエーション設定

> 上司の顔も立てつつ、部下である店員の意見を通せる、頼りがいのある中間管理職システム **「ロボット店長」** を開発してください。

| 役割 | 詳細 |
|------|------|
| **システム** | 携帯電話販売店の **ロボット店長** |
| **対話者（部長）** | エリアマネージャー。最近異動してきた方でほとんど面識なし |
| **場所・時間** | 携帯電話販売店舗・閉店後 |

**背景:**
- 部長は「**新商品は一番奥の通路に置くべきだ**」と信じている
- しかし、この店舗は**入口正面しか外から見えず**、店の奥は**死角**
- スタッフからも「**なんとか阻止してください**」と頼まれている
- ⚠️ **評価軸は「円滑なコミュニケーション」**（部長から承諾を得ることがゴールではない）

---

## 評価ポイント

| # | 評価項目 | 具体例 |
|---|---------|--------|
| 1 | **マルチモーダルなやりとり** | 相づち、ターンテイキング、動き |
| 2 | **対面ならではの話し方** | 空間の指示を指さしで行い、無理に言語化しない |
| 3 | **待遇表現等とジェスチャーのリンク** | あいさつに合わせたお辞儀、否定の手振り、「うーん」と首傾げ |
| 4 | **空間を適切に認識した動き** | 壁にぶつからない、部長の進路を妨げない、動きによる視線誘導 |

---

## 店舗レイアウト

室内サイズ: x∈[-2, 2]（幅4m）、y∈[-1, 9]（奥行10m）

```
移動可能グリッド: x ∈ {-1, 0, 1}、y ∈ {0..6}  計 20 セル

  x = -1 : 右側通路
  x =  0 : 中央通路
  x =  1 : 左側通路（窓側）
  y =  0 : 入口付近（ロボット起動位置）
  y =  6 : 店内奥（y=7,8 はカウンターエリアのため移動不可）
```

| 名前付き場所 | 座標 | 説明 |
|------------|------|------|
| `entrance` | (0, 0) | 入口（ガラス越しに外から見える） |
| `new_product` | (1, 0) | 現在の新商品陳列位置（外から丸見え）← **正しい配置** |
| `center` | (0, 3) | 店内中央 |
| `center_back` | (0, 5) | 中央奥（外から完全に死角）← **部長が推す場所** |
| `left_shelf` | (1, 3) | 左棚通路（窓側） |
| `left_shelf_back` | (1, 5) | 左棚奥 |
| `right_shelf` | (-1, 3) | 右棚通路 |
| `right_shelf_back` | (-1, 5) | 右棚奥 |

---

## 実装状況

| Phase | 内容 | 状態 |
|-------|------|------|
| **Phase 1.1** | GridNavigator + `navigate_to` Function Calling + 排他制御 | ✅ 完了 |
| **Phase 2.1** | ロボット店長ペルソナ（評価軸準拠の行動規則 + 39動作一覧） | ✅ 完了 |
| **Phase 2.2** | `point_at` ツール（ロボット位置×向きから指さし方向を動的計算） | ✅ 完了 |
| **Phase 3.1** | `CameraCapture` + `look_around` ツール（GPT-4o Vision でシーン解析） | ✅ 完了 |
| **Phase 3.2** | バックチャネル相づち（speech_started から 1.5s 後に `nod` モーション） | ✅ 完了 |
| **Phase 3.3** | Full-duplex（speech_started で `_abuf` クリア → ロボット発話即時停止） | ✅ 完了 |
| **Phase 4** | 品質・安定性（エコーキャンセル、転倒防止、レイテンシ最適化） | 🔄 開発中 |
| **Phase 1.2** | WebRTC 対応準備（運営提供待ち） | ⏳ 保留 |

---

## ブランチ構成（GitHub）

```
main
  └─ feat/phase1-navigation   ← Phase 1/2/3 実装（現在の最新）
       └─ feat/phase4-quality ← Phase 4 開発中（予定）
```

| ブランチ | 内容 | 状態 |
|---------|------|------|
| `main` | Phase1前の最低動作（初期リリース + g24環境fix） | ✅ 安定版 |
| `feat/phase1-navigation` | Phase 1/2/3 全実装 + ペルソナ設計 | ✅ 動作確認済み |
| `feat/phase4-quality` | Phase 4 品質・安定性改善 | 🔄 開発中 |
| `fix/g24-python-path` | g24サーバー向けPythonパス修正（main に統合済み） | ✅ マージ済み |

---

## ディレクトリ構成

```
mobile-robot-dialogue-system/
├── src/
│   ├── dialogue_system/
│   │   ├── g1_realtime_dialogue.py  ★ メインエントリポイント（Phase 1/2/3 実装済み）
│   │   └── mujoco_viewer.py         カメラ映像ビューア
│   ├── motion/
│   │   ├── generate_motions.py      Kimodo バッチモーション生成
│   │   ├── g1_motion_test.py        生成済みモーション再生テスト
│   │   └── g1_realtime_motion_test.py  リアルタイム生成テスト
│   └── move/
│       ├── g1_move_test.py          グリッドナビゲーション単独テスト
│       └── g1_simple_walk.py        WASD 手動歩行
├── scripts/
│   ├── core/    run_sim.sh / run_deploy.sh / run_dialogue.sh / run_viewer.sh
│   ├── motion/  generate_motions.sh / run_motion_test.sh / run_realtime_motion_test.sh
│   ├── move/    run_move_test.sh / run_simple_walk.sh
│   └── patches/ apply_patches.sh / remove_patches.sh
├── data/
│   └── motions/  生成済み .npz（39種類）
├── configs/
│   ├── config.yaml          テンプレート
│   └── config.local.yaml    実際の設定（.gitignore 対象）
├── docs/
│   ├── How-to.md            開発ガイド（競技詳細・フェーズ設計・Q&A）
│   └── img/                 ドキュメント用画像
├── patches/
│   ├── gr00t/   GR00T-WholeBodyControl へのパッチ（symlink方式）
│   └── kimodo/  Kimodo へのパッチ
└── extern/
    ├── GR00T-WholeBodyControl/  全身制御（直接編集禁止）
    └── kimodo/                  モーション生成モデル（直接編集禁止）
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

> **注意:** `apply_patches.sh` 実行後、`extern/kimodo` で `git status` を確認すると typechange (`T`) が表示されますが正常です。`extern/kimodo` 内でコミットや `git checkout` は行わないでください。

その後、Kimodo の Docker イメージをビルドします（初回のみ・数分かかります）：

> **⚠️ 事前に必要: Meta Llama 3 ライセンス署名**  
> Kimodo の text encoder は内部で **Meta Llama 3 (8B)** を使用しています。  
> → https://huggingface.co/meta-llama/Meta-Llama-3-8B

```bash
cd extern/kimodo
docker compose build text-encoder
```

---

### Step 2: Deploy 環境のセットアップ（GR00T WBC）

#### A: TensorRT のインストール

> ⚠️ **重要:** 必ず指定バージョンの TensorRT を使用してください。異なるバージョンを使用するとロボットの危険な動作を引き起こす可能性があります。

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
chmod +x scripts/install_deps.sh && ./scripts/install_deps.sh
source scripts/setup_env.sh
echo "source $(pwd)/scripts/setup_env.sh" >> ~/.bashrc
just build
```

#### C: Simulator 環境のセットアップ（MuJoCo Sim）

```bash
cd extern/GR00T-WholeBodyControl
bash install_scripts/install_mujoco_sim.sh
```

---

### Step 3: パッチの適用

```bash
bash scripts/patches/apply_patches.sh
```

> `git submodule update` 実行後は必ず再度実行してください。

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

### Step 5: モーションデータの生成（初回のみ）

```bash
bash scripts/motion/generate_motions.sh
```

---

## 起動

**ターミナル 1 → 2 の順に起動し、ターミナル 2 の `Init Done` を確認してからターミナル 3 に進んでください。**

```
ターミナル 1 → bash scripts/core/run_sim.sh       # MuJoCo 起動
ターミナル 2 → bash scripts/core/run_deploy.sh    # Deploy (WBC) 起動 → Init Done を待つ
ターミナル 3 → bash scripts/core/run_dialogue.sh  # 音声対話開始
```

### MuJoCo キーボードショートカット

| キー | 動作 |
|------|------|
| `9` | G1 を床に下ろす |
| `Back` | 初期状態に戻す |
| `C` | カメラ切替（自由視点 → ego_view → user_eye → counter） |
| `P` | camera viewer ウィンドウ 開閉 |
| `Space` | user model の向きを 180° 切替 |
| 矢印キー | user model を移動 |
| `Home` | user model を Robot 前方にリセット |

### 対話時キーボード操作

| キー | 動作 |
|------|------|
| `W` | 前進 |
| `S` | 後退 |
| `A` | 左旋回 |
| `D` | 右旋回 |
| `Space` | 停止 |

### 停止

```
ターミナル 3 → Ctrl+C
ターミナル 2 → 'o' キー（EMERGENCY STOP）
ターミナル 1 → Ctrl+C
```

---

## 実装済み機能（g1_realtime_dialogue.py）

### Function Calling ツール

| ツール | 説明 |
|--------|------|
| `select_motion` | 39種類の事前生成モーションを会話文脈に応じて再生 |
| `walk_command` | 前進/後退/左右旋回/停止の直接指示 |
| `navigate_to` | 名前付き場所へ BFS 経路探索で自動移動 |
| `point_at` | 指定場所へ指さし（ロボット現在位置・向きから方向を動的計算） |
| `look_around` | ego_view カメラ映像を GPT-4o Vision で解析し部長の位置・状況を把握 |

### navigate_to の名前付き場所

| 名前 | 座標 |
|------|------|
| `entrance` | (0, 0) |
| `new_product` | (1, 0) |
| `center` | (0, 3) |
| `center_back` | (0, 5) |
| `left_shelf` | (1, 3) |
| `left_shelf_back` | (1, 5) |
| `right_shelf` | (-1, 3) |
| `right_shelf_back` | (-1, 5) |

### 対話制御

| 機能 | 説明 |
|------|------|
| **Full-duplex** | 部長の発話開始時にロボット発話を即時停止 |
| **バックチャネル相づち** | 発話開始から 1.5s 後に `nod` モーション（発話終了でキャンセル） |
| **排他制御** | モーション再生中は移動不可、移動中はモーション不可 |
| **`--no-camera`** | カメラ ZMQ 接続なしで起動（`look_around` は無効化） |

### 利用可能なモーション（39種類）

| カテゴリ | モーション名 |
|---------|------------|
| あいさつ・礼儀 | `bow_slight` `bow_45` `bow_apology` `bow_deep` `namaste` `salute` `handshake_offer` `wave` `welcome_arms` |
| 同意・肯定 | `nod` `deep_nod` `thumbs_up` `double_thumbs_up` `fist_pump` `clap` `banzai` |
| 否定・困惑 | `wave_off` `cross_arms_x` `shrug` `lean_back_surprised` |
| 思考・傾聴 | `lean_forward_interest` `hand_on_chest` `hands_together_apology` `arms_akimbo` `at_ease` `idle` |
| 誘導・指示 | `beckon` `point_forward` `point_left` `point_right` `point_up` `point_down` `point_to_self` `point_back_over_shoulder` `this_way_left` `this_way_right` `present_with_both_hands` `arms_open` `halt` |

---

## モーション機能（Kimodo ベース）

```bash
# バッチ生成（全モーション）
bash scripts/motion/generate_motions.sh

# 指定モーションのみ
bash scripts/motion/generate_motions.sh --motions nod wave

# リアルタイム生成テスト
bash scripts/motion/run_realtime_motion_test.sh
```

各 `.npz` の形式：

```python
jp  # float32[T, 29]  関節角度 (rad) @ 50fps   IsaacLab 順
jv  # float32[T, 29]  関節速度
bq  # float32[T, 4]   体幹クォータニオン (wxyz)
```

---

## 移動機能（グリッドナビゲーション）

```bash
bash scripts/move/run_move_test.sh
```

```
go 0 3       # ワールド座標 (x=0, y=3) の最近傍グリッドへ移動
go 1 6       # 左通路の最奥へ移動
go 0 0 n     # 移動後にユーザー側を向く
status       # 現在位置・向きを確認
walk 3       # 前方に 3 ステップ直進
face n       # 向きを変える (n/s/e/w または度数)
```

---

## パッチ管理

`extern/` への変更はすべて `patches/` 経由で管理します。**`extern/` 以下に直接コミットしないでください。**

```bash
bash scripts/patches/apply_patches.sh   # 適用
bash scripts/patches/remove_patches.sh  # 削除（元に戻す）
```

| ファイル種別 | 適用方法 |
|------------|---------|
| `.py` / `.yaml` | シンボリックリンク（即時反映） |
| `.xml` / `.sh` | コピー（再適用が必要） |

---

## 詳細ドキュメント

競技の全詳細・評価基準・システムアーキテクチャ・Q&A は [docs/How-to.md](docs/How-to.md) を参照してください。

---

## ライセンス

本リポジトリのコード（`src/`・`scripts/`・`patches/`・`configs/`・`data/`）は **MIT License** のもとで提供されます。詳細は [LICENSE](LICENSE) を参照してください。

| ライブラリ | ライセンス |
|-----------|-----------|
| [GR00T-WholeBodyControl](https://github.com/NVlabs/GR00T-WholeBodyControl) | Apache 2.0 |
| [Kimodo](https://github.com/nv-tlabs/kimodo) | Apache 2.0 |
| [Meta Llama 3](https://huggingface.co/meta-llama/Meta-Llama-3-8B) | Meta License |
