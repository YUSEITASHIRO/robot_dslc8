#!/usr/bin/env python3
"""
generate_motions.py — Kimodo G1 動作バッチ生成スクリプト

生成した動作を以下のパスに保存します:
  data/motions/<name>/sample_1.npz
                      sample_2.npz
                      sample_3.npz

各 npz ファイルの内容:
  jp  float32[T, 29]  関節角度 (rad) at 50fps
  jv  float32[T, 29]  関節速度
  bq  float32[T, 4]   体幹クォータニオン wxyz

開始・終了は自然な直立姿勢 (jp=0, bq=[1,0,0,0]) と
スムーズに接続するよう多セグメントプロンプト＋境界ブレンドで処理します。

実行環境: base conda (kimodo インストール済み)

Usage:
    python src/motion/generate_motions.py
    python src/motion/generate_motions.py --motions nod wave bow_slight
    python src/motion/generate_motions.py --duration 3
    python src/motion/generate_motions.py --samples 1 --steps 50
    python src/motion/generate_motions.py --list
"""

import argparse
import os
import sys

import numpy as np
from scipy.interpolate import interp1d
from scipy.spatial.transform import Rotation, Slerp

sys.path.insert(0, "/home/unitree-g1/Documents/G1/kimodo")

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
OUTPUT_DIR = os.path.join(REPO_ROOT, "data", "motions")

MODEL_NAME     = "kimodo-g1-rp"
KIMODO_FPS     = 30          # G1モデルの出力fps
SONIC_FPS      = 50          # SONIC / 対話システムが要求するfps
TRANSITION_DUR = 0.5         # 開始・終了の「直立」セグメント (秒)
BLEND_FRAMES   = 12          # 境界ブレンドフレーム数 (50fps, ~0.24s)
N_SAMPLES      = 1
DURATION       = 5.0
DIFFUSION_STEPS = 100

# ── 関節順序変換 (Kimodo → SONIC ZMQ) ─────────────────────────
# Kimodo の dict_to_qpos は MuJoCo 順 (左右脚→腰→左腕→右腕) で jp を出力する。
# SONIC deploy の output_interface.hpp は ZMQ joint_pos を IsaacLab 順として
# body_q_target[mj] = ZMQ[isaaclab_to_mujoco[mj]] と展開する。
# よって保存前に Kimodo(MuJoCo) 順 → IsaacLab 順へ並び替えが必要。
# (policy_parameters.hpp の mujoco_to_isaaclab 配列と同一)
MUJOCO_TO_ISAACLAB = np.array([
     0,  6, 12,  1,  7, 13,  2,  8, 14,
     3,  9, 15, 22,  4, 10, 16, 23,
     5, 11, 17, 24, 18, 25, 19, 26,
    20, 27, 21, 28
], dtype=np.int32)

# ── 直立姿勢 ──────────────────────────────────────────────────
# IsaacLab 順でのゼロベクトル。腕は 0 rad ≈ 自然下垂位置 (OK)。
# 脚もゼロだが，Kimodo の STAND_PROMPT セグメントが実際の遷移を担うため
# BLEND_FRAMES (0.24s) の短いフェードイン/アウトとして許容範囲。
STANDING_JP  = np.zeros(29, dtype=np.float32)
STANDING_BQ  = np.array([1., 0., 0., 0.], dtype=np.float32)  # wxyz

STAND_PROMPT = (
    "A humanoid robot standing completely still in a natural upright position, "
    "arms relaxed at sides, feet together."
)

# ── 動作定義 ───────────────────────────────────────────────────
# prompt はKimodo が受け取る英語テキスト (mid-segment)
MOTIONS: dict[str, dict] = {
    # 待機・基本
    "idle": {
        "desc": "直立待機",
        "prompt": (
            "A humanoid robot standing completely still in a natural upright position, "
            "arms relaxed at sides."
        ),
    },
    "at_ease": {
        "desc": "休憩姿勢（アットイーズ）",
        "prompt": (
            "A humanoid robot in a relaxed at-ease posture, weight shifted slightly, "
            "hands clasped loosely behind the back."
        ),
    },
    # 肯定・共感
    "nod": {
        "desc": "うなずき（腰pitch）",
        "prompt": (
            "A humanoid robot nodding its head up and down gently in agreement, "
            "a subtle forward inclination of the upper body."
        ),
    },
    "deep_nod": {
        "desc": "大きくうなずき",
        "prompt": (
            "A humanoid robot performing a deep, exaggerated nod of strong agreement, "
            "bowing the head significantly while keeping feet in place."
        ),
    },
    "thumbs_up": {
        "desc": "親指を立てる（右手）",
        "prompt": (
            "A humanoid robot raising its right hand with thumb pointing straight up "
            "in a clear thumbs-up gesture of approval, arm extended forward and upward."
        ),
    },
    "double_thumbs_up": {
        "desc": "両手で親指を立てる",
        "prompt": (
            "A humanoid robot raising both hands simultaneously with thumbs pointing up "
            "in a double thumbs-up gesture of enthusiastic approval."
        ),
    },
    "hand_on_chest": {
        "desc": "胸に手を当てる（誠意・自己紹介）",
        "prompt": (
            "A humanoid robot placing its right hand flat on its chest in a sincere, "
            "heartfelt gesture of self-introduction or commitment."
        ),
    },
    "point_to_self": {
        "desc": "自分を指す",
        "prompt": (
            "A humanoid robot pointing at itself with its right index finger, "
            "indicating itself clearly."
        ),
    },
    # 否定・困惑
    "wave_off": {
        "desc": "手を左右に振って否定",
        "prompt": (
            "A humanoid robot waving its right hand side to side in a dismissive "
            "wave-off gesture, indicating no or refusal."
        ),
    },
    "cross_arms_x": {
        "desc": "両腕を胸前でXにする（不可・ダメ）",
        "prompt": (
            "A humanoid robot crossing both forearms in front of its chest forming "
            "an X shape, indicating no, stop, or not allowed."
        ),
    },
    "lean_back_surprised": {
        "desc": "腰を後ろに反らせて驚き",
        "prompt": (
            "A humanoid robot leaning its upper body backward with a startled, "
            "surprised reaction, arms slightly spread outward."
        ),
    },
    # 挨拶
    "bow_slight": {
        "desc": "軽いお辞儀（15度）",
        "prompt": (
            "A humanoid robot bowing its head and upper body forward at about 15 degrees "
            "in a polite acknowledgment or light greeting."
        ),
    },
    "bow_45": {
        "desc": "中間のお辞儀（45度）",
        "prompt": (
            "A humanoid robot bowing forward at about 45 degrees in a respectful greeting, "
            "holding the bow briefly."
        ),
    },
    "bow_deep": {
        "desc": "深いお辞儀",
        "prompt": (
            "A humanoid robot performing a deep bow, bending the upper body far forward "
            "in a deeply respectful greeting."
        ),
    },
    "wave": {
        "desc": "右手を振る（挨拶）",
        "prompt": (
            "A humanoid robot raising its right hand to shoulder height and waving "
            "it back and forth in a friendly greeting."
        ),
    },
    "salute": {
        "desc": "敬礼",
        "prompt": (
            "A humanoid robot raising its right hand to its forehead with fingers together "
            "in a formal military salute."
        ),
    },
    "namaste": {
        "desc": "胸前で合掌（ナマステ）",
        "prompt": (
            "A humanoid robot bringing both palms together in front of its chest "
            "in a namaste or prayer greeting gesture, with a slight forward bow."
        ),
    },
    "handshake_offer": {
        "desc": "右手を前に差し出す（握手）",
        "prompt": (
            "A humanoid robot extending its right arm forward with an open hand, "
            "offering a handshake."
        ),
    },
    "welcome_arms": {
        "desc": "両腕を斜め前に開いて歓迎",
        "prompt": (
            "A humanoid robot opening both arms wide forward and outward "
            "in a warm welcoming gesture."
        ),
    },
    # 指示・案内
    "point_forward": {
        "desc": "前方を指差す",
        "prompt": (
            "A humanoid robot extending its right arm straight forward and pointing "
            "ahead with its index finger."
        ),
    },
    "point_left": {
        "desc": "左を指差す",
        "prompt": (
            "A humanoid robot extending its left arm to the left and pointing "
            "in that direction with its index finger."
        ),
    },
    "point_right": {
        "desc": "右を指差す",
        "prompt": (
            "A humanoid robot extending its right arm to the right and pointing "
            "in that direction with its index finger."
        ),
    },
    "point_up": {
        "desc": "上方を指差す",
        "prompt": (
            "A humanoid robot raising its right arm upward and pointing toward "
            "the ceiling with its index finger."
        ),
    },
    "point_down": {
        "desc": "足元・床を指差す",
        "prompt": (
            "A humanoid robot lowering its right arm and pointing downward toward "
            "the floor with its index finger."
        ),
    },
    "point_back_over_shoulder": {
        "desc": "肩越しに後方を指す",
        "prompt": (
            "A humanoid robot reaching its right arm over its shoulder to point "
            "backward behind itself with its thumb or index finger."
        ),
    },
    "arms_open": {
        "desc": "両腕を広げて案内",
        "prompt": (
            "A humanoid robot spreading both arms wide to the sides with palms facing "
            "forward in a grand presenting or welcoming gesture."
        ),
    },
    "this_way_right": {
        "desc": "右手のひらを上に、右方向へ案内",
        "prompt": (
            "A humanoid robot extending its right arm to the right with palm facing "
            "upward, guiding someone to the right."
        ),
    },
    "this_way_left": {
        "desc": "左手のひらを上に、左方向へ案内",
        "prompt": (
            "A humanoid robot extending its left arm to the left with palm facing "
            "upward, guiding someone to the left."
        ),
    },
    "present_with_both_hands": {
        "desc": "両手のひらを上に、丁寧に提示",
        "prompt": (
            "A humanoid robot extending both hands forward with palms facing upward "
            "in a polite presenting or offering gesture."
        ),
    },
    "beckon": {
        "desc": "手招き（こちらへどうぞ）",
        "prompt": (
            "A humanoid robot raising one hand with palm facing inward and curling "
            "the fingers repeatedly to beckon someone to come closer."
        ),
    },
    "halt": {
        "desc": "手のひらを正面に立てて「お待ちください」",
        "prompt": (
            "A humanoid robot raising one hand with palm facing outward flat, "
            "signaling to stop or wait."
        ),
    },
    # 感情表現
    "shrug": {
        "desc": "肩をすくめる",
        "prompt": (
            "A humanoid robot shrugging both shoulders upward while spreading "
            "both hands outward in an uncertain, don't-know gesture."
        ),
    },
    "clap": {
        "desc": "拍手",
        "prompt": (
            "A humanoid robot clapping both hands together repeatedly in front "
            "of its chest in applause."
        ),
    },
    "banzai": {
        "desc": "両手を上に万歳",
        "prompt": (
            "A humanoid robot raising both arms straight up above its head "
            "in a banzai or victory celebration gesture."
        ),
    },
    "fist_pump": {
        "desc": "ガッツポーズ",
        "prompt": (
            "A humanoid robot making a fist with its right hand and pumping the arm "
            "downward in a fist-pump victory gesture."
        ),
    },
    "lean_forward_interest": {
        "desc": "前傾して興味を示す",
        "prompt": (
            "A humanoid robot leaning its upper body slightly forward with arms "
            "loosely at sides, showing interest and attentiveness."
        ),
    },
    "arms_akimbo": {
        "desc": "腰に手を当てる（アキンボ）",
        "prompt": (
            "A humanoid robot placing both hands on its hips with elbows pointing "
            "outward in an akimbo confident posture."
        ),
    },
    # 謝罪
    "bow_apology": {
        "desc": "深く長めの謝罪お辞儀",
        "prompt": (
            "A humanoid robot bowing deeply with its upper body for several seconds "
            "in a sincere and prolonged apology bow."
        ),
    },
    "hands_together_apology": {
        "desc": "両手を前で合わせ頭を下げる（謝罪）",
        "prompt": (
            "A humanoid robot bringing both hands together in front of its body "
            "while bowing its head downward in an apologetic gesture."
        ),
    },
}


# ── ユーティリティ ─────────────────────────────────────────────

def resample(arr: np.ndarray, src_fps: float, dst_fps: float) -> np.ndarray:
    """1D/2D 配列を src_fps から dst_fps へ線形補間でリサンプル。"""
    T = arr.shape[0]
    T_new = max(2, int(round(T * dst_fps / src_fps)))
    t_src = np.linspace(0.0, 1.0, T)
    t_dst = np.linspace(0.0, 1.0, T_new)
    return interp1d(t_src, arr, axis=0, kind="linear")(t_dst).astype(arr.dtype)


def slerp_quat(q_start: np.ndarray, q_end: np.ndarray, n: int) -> np.ndarray:
    """wxyz クォータニオン間の SLERP (n フレーム 0→1)。"""
    # scipy は xyzw なので変換
    r_start = Rotation.from_quat(q_start[[1, 2, 3, 0]])
    r_end   = Rotation.from_quat(q_end[[1, 2, 3, 0]])
    times   = np.linspace(0.0, 1.0, n)
    rots    = Slerp([0.0, 1.0], Rotation.concatenate([r_start, r_end]))(times)
    xyzw    = rots.as_quat()
    return np.column_stack([xyzw[:, 3], xyzw[:, :3]]).astype(np.float32)  # wxyz


def blend_boundaries(jp: np.ndarray, bq: np.ndarray, n: int) -> tuple[np.ndarray, np.ndarray]:
    """先頭 n フレームと末尾 n フレームを直立姿勢へブレンド。"""
    jp = jp.copy()
    bq = bq.copy()
    # 先頭: 直立 → 生成値
    alpha = np.linspace(0.0, 1.0, n, dtype=np.float32)[:, None]
    jp[:n] = (1 - alpha) * STANDING_JP + alpha * jp[:n]
    bq[:n] = slerp_quat(STANDING_BQ, bq[n - 1], n)
    # 末尾: 生成値 → 直立
    alpha = np.linspace(0.0, 1.0, n, dtype=np.float32)[:, None]
    jp[-n:] = (1 - alpha) * jp[-n] + alpha * STANDING_JP
    bq[-n:] = slerp_quat(bq[-(n + 1)], STANDING_BQ, n)
    return jp, bq


def compute_jv(jp: np.ndarray) -> np.ndarray:
    jv = np.zeros_like(jp)
    if len(jp) > 1:
        jv[:-1] = (jp[1:] - jp[:-1]) * SONIC_FPS
        jv[-1]  = jv[-2]
    return jv


def save_npz(path: str, jp: np.ndarray, bq: np.ndarray) -> None:
    jv = compute_jv(jp)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    np.savez(path, jp=jp, jv=jv, bq=bq)


# ── 生成 ───────────────────────────────────────────────────────

def build_segments(motion_prompt: str, dur: float):
    """多セグメントプロンプトと各セグメントのフレーム数を返す。"""
    core_dur = max(0.5, dur - 2 * TRANSITION_DUR)
    tr_frames   = int(round(TRANSITION_DUR * KIMODO_FPS))
    core_frames = int(round(core_dur * KIMODO_FPS))
    texts = [
        STAND_PROMPT,
        motion_prompt,
        STAND_PROMPT,
    ]
    num_frames = [tr_frames, core_frames, tr_frames]
    return texts, num_frames


def generate_and_save(
    model,
    converter,
    device: str,
    name: str,
    motion_info: dict,
    duration: float,
    n_samples: int,
    diffusion_steps: int,
    skip_existing: bool,
) -> None:
    texts, num_frames = build_segments(motion_info["prompt"], duration)

    # スキップ確認
    if skip_existing:
        all_done = all(
            os.path.exists(os.path.join(OUTPUT_DIR, name, f"sample_{i+1}.npz"))
            for i in range(n_samples)
        )
        if all_done:
            print(f"  スキップ (既存)")
            return

    print(f"  生成中… (core {sum(num_frames[1:2])/KIMODO_FPS:.1f}s, {n_samples}samples)", flush=True)

    output = model(
        prompts=texts,
        num_frames=num_frames,
        num_denoising_steps=diffusion_steps,
        multi_prompt=True,
        num_samples=n_samples,
    )

    qpos = converter.dict_to_qpos(output, device)  # (B, T, 36)
    qpos_np = qpos.cpu().numpy() if hasattr(qpos, "cpu") else np.array(qpos)

    for i in range(n_samples):
        q = qpos_np[i]            # (T_30fps, 36)
        jp_30 = q[:, 7:].astype(np.float32)   # (T, 29) Kimodo MuJoCo 順
        bq_30 = q[:, 3:7].astype(np.float32)  # (T, 4) wxyz

        # Kimodo(MuJoCo) 順 → IsaacLab 順 (SONIC ZMQ 期待形式)
        jp_30 = jp_30[:, MUJOCO_TO_ISAACLAB]

        # 30fps → 50fps リサンプル
        jp_50 = resample(jp_30, KIMODO_FPS, SONIC_FPS)
        bq_50 = resample(bq_30, KIMODO_FPS, SONIC_FPS)
        # クォータニオン正規化
        bq_50 /= np.linalg.norm(bq_50, axis=1, keepdims=True).clip(1e-8)

        # 境界ブレンド (脚・腰のみ; 腕は Kimodo 値を保持)
        # G1 のアーム関節は 0 rad = 腕が横に上がった姿勢になるため，
        # STANDING_JP (zeros) への blend は脚・腰のみに適用し，
        # 腕関節は Kimodo 生成値をそのまま使用する。
        ARM_IL = np.array([11, 12, 15, 16, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28], dtype=np.int32)
        blend_n = min(BLEND_FRAMES, len(jp_50) // 4)
        arm_jp_orig = jp_50[:, ARM_IL].copy()   # 腕関節を退避
        jp_50, bq_50 = blend_boundaries(jp_50, bq_50, blend_n)
        jp_50[:, ARM_IL] = arm_jp_orig          # 腕関節を復元 (ブレンド対象外)

        out_path = os.path.join(OUTPUT_DIR, name, f"sample_{i+1}.npz")
        save_npz(out_path, jp_50, bq_50)
        print(f"    sample_{i+1} → {out_path} ({len(jp_50)}frames @ {SONIC_FPS}fps)")


# ── メイン ─────────────────────────────────────────────────────

def parse_args():
    parser = argparse.ArgumentParser(description="Kimodo G1 動作バッチ生成")
    parser.add_argument("--motions", nargs="*", default=None,
                        help="生成する動作名 (省略時: 全動作)")
    parser.add_argument("--duration", type=float, default=DURATION,
                        help=f"生成秒数 (デフォルト: {DURATION})")
    parser.add_argument("--samples", type=int, default=N_SAMPLES,
                        help=f"1動作あたりのサンプル数 (デフォルト: {N_SAMPLES})")
    parser.add_argument("--steps", type=int, default=DIFFUSION_STEPS,
                        help=f"Diffusion ステップ数 (デフォルト: {DIFFUSION_STEPS}, 速度優先なら 50)")
    parser.add_argument("--skip-existing", action="store_true", default=True,
                        help="既存 npz をスキップ (デフォルト: True)")
    parser.add_argument("--no-skip", dest="skip_existing", action="store_false",
                        help="既存 npz を上書き")
    parser.add_argument("--list", action="store_true",
                        help="動作一覧を表示して終了")
    return parser.parse_args()


def main():
    args = parse_args()

    if args.list:
        print(f"{'名前':<36} 説明")
        print("-" * 60)
        for name, info in MOTIONS.items():
            print(f"  {name:<34} {info['desc']}")
        return

    target_motions = args.motions or list(MOTIONS.keys())
    unknown = [m for m in target_motions if m not in MOTIONS]
    if unknown:
        print(f"[ERROR] 不明な動作名: {unknown}")
        sys.exit(1)

    print(f"[Setup] Kimodo モデル読み込み中: {MODEL_NAME}")
    import torch
    from kimodo import load_model
    from kimodo.exports.mujoco import MujocoQposConverter

    device = "cuda:0" if torch.cuda.is_available() else "cpu"
    print(f"[Setup] デバイス: {device}")

    model, resolved = load_model(MODEL_NAME, device=device, return_resolved_name=True)
    print(f"[Setup] ロード完了: {resolved}  fps={model.fps}")

    converter = MujocoQposConverter(model.skeleton)

    total = len(target_motions)
    done  = 0
    for name in target_motions:
        info = MOTIONS[name]
        done += 1
        print(f"\n[{done}/{total}] {name}  — {info['desc']}")
        generate_and_save(
            model=model,
            converter=converter,
            device=device,
            name=name,
            motion_info=info,
            duration=args.duration,
            n_samples=args.samples,
            diffusion_steps=args.steps,
            skip_existing=args.skip_existing,
        )

    print("\n=== 生成完了 ===")
    print(f"保存先: {OUTPUT_DIR}")


if __name__ == "__main__":
    main()
