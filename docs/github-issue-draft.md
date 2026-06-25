# [進捗報告] Phase 1〜7 完了・Phase 6.2 保留

## 概要

2026-06-25 時点の実装進捗と残課題をまとめます。

---

## ✅ 完了済み（Phase 1〜7）

| Phase | 内容 | ブランチ |
|-------|------|---------|
| **1.1** | GridNavigator + `navigate_to` Function Calling + streaming/planner 排他制御 | feat/phase1-navigation |
| **2.1** | ロボット店長ペルソナ（評価軸準拠の行動規則 + 39動作一覧） | feat/phase1-navigation |
| **2.2** | `point_at` ツール（ロボット位置×向きから指さし方向を動的計算） | feat/phase1-navigation |
| **3.1** | `CameraFrameSubscriber` + `look_around` ツール（GPT-4o Vision でシーン解析） | feat/phase1-navigation |
| **3.2** | バックチャネル相づち（speech_started から 1.5s 後に `nod` モーション） | feat/phase1-navigation |
| **3.3** | Full-duplex（speech_started で `_abuf` クリア → ロボット発話即時停止） | feat/phase1-navigation |
| **4.1** | 転倒防止 watchdog（navigate_to タイムアウト 60s 強制停止） | feat/phase4-quality |
| **4.2** | レイテンシ最適化（VAD パラメータ定数化・TURN 値を運営推奨値に統一） | feat/phase4-quality |
| **4.3** | 運営提供バージョン同期（WSL 安定化・別スレッドレンダリング・CameraFrameSubscriber 移行） | feat/phase4-quality |
| **5.1** | エコーキャンセル（音声ゲート方式 — TTS 再生中はマイク送信を停止 + 400ms クールダウン） | feat/phase5-enhancement |
| **5.2** | シーンキャッシュ（TTL 30s）+ 接続後 3s で初回自動スキャン | feat/phase5-enhancement |
| **6.1** | 会話フェーズ追跡（GREETING→TOURING→NEGOTIATING→CLOSING）+ `session.update` で動的 instructions 切替 | feat/phase6-strategy |
| **7.1** | バックグラウンドシーン監視（25s 間隔で Vision スキャン・部長の動き検出） | feat/phase6-strategy |
| **7.2** | 対話者位置追従ナビゲーション（Vision 結果から奥エリア移動を検出して先回り誘導） | feat/phase6-strategy |
| **6.2** | `AudioBackend` 抽象化 + `WebRTCAudioBackend`（aiortc + aiohttp）、`--webrtc` フラグで切替 | feat/phase6.2-webrtc |

---

## 🔲 残課題・未実装

### 中優先度

- **Phase 6.2 本実装: 運営 WebRTC 仕様への対応**
  - 現在は aiortc + aiohttp による仮実装（`--webrtc` フラグで起動）
  - `WebRTCAudioBackend` のみ差し替えれば OK な構造にしてあるため、仕様確定後に対応

### 低優先度

- **リアルタイムモーション生成の統合**（Kimodo オンデマンド生成を対話中に使用）
- **Windows 対応**（`termios` → クロスプラットフォーム制御）

---

## 既知の技術的課題

| 課題 | 詳細 | 回避策 |
|------|------|--------|
| VAD 誤検出 | エコーゲート期間外でも TTS の残響が speech_started を引き起こす可能性 | ECHO_GATE_COOLDOWN_S を延長（現在 0.40s） |
| look_around レイテンシ | GPT-4o Vision API 呼び出しで 2〜4s かかる | シーンキャッシュで軽減（Phase 5.2） |
| 旋回精度 | TURN_REPEAT_COUNT=5 でも環境によっては角度がずれる | TURN_STEP_DEG で微調整 |
| WebRTC 未対応 | 予選・本選時の音声通信方式が異なる | 運営提供仕様待ち（Phase 6.2） |
| フェーズ誤遷移 | キーワード検出が誤検出するケースあり（「奥」が別文脈で出現等） | 閾値調整・ターン数との組み合わせで抑制 |

---

## スケジュール

| 締切 | イベント |
|------|---------|
| 2026/09/01 | システム開発締切 |
| 2026/09/12 | 予選開催 |
| 2026/11/14 | 本選（早稲田大学） |

---

## 次のアクション

1. `feat/phase5-enhancement` の PR を作成して `main` にマージ
2. `feat/phase6-strategy` の PR を作成して `main` にマージ
3. 運営から WebRTC 仕様が届いたら Phase 6.2 に対応

/label: progress, phase7, planning
