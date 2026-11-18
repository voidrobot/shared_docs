#!/usr/bin/env python3
"""
Analyzes 30 Hz policy output (`target_joint10..target_joint43`) in `_policy.csv`,
generates velocity STFT spectrograms (0..15 Hz), and detects >= 3.0 s high-frequency
oscillations (>= 3.0 Hz) to verify whether oscillations originate in the policy
before Command EMA / rate-limit post-processing.
"""

import argparse
import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import numpy as np
from scipy import interpolate, signal


FINGER_GROUPS = [
    ("Index", ["target_joint10", "target_joint11", "target_joint12", "target_joint13"]),
    ("Middle", ["target_joint20", "target_joint21", "target_joint22", "target_joint23"]),
    ("Ring", ["target_joint30", "target_joint31", "target_joint32", "target_joint33"]),
    ("Pinky", ["target_joint40", "target_joint41", "target_joint42", "target_joint43"]),
]


def load_policy_non_thumb(csv_path: Path, target_hz: float = 30.0) -> tuple[float, float, list[str], np.ndarray]:
    """Loads `_policy.csv` and resamples non-uniform `t` timestamps onto an exact `target_hz` grid."""
    with csv_path.open("r", newline="", encoding="utf-8") as f:
        reader = csv.reader(f)
        header = next(reader)
        col_idx = {name: i for i, name in enumerate(header)}
        target_names = [f"target_joint{fg}{j}" for fg in range(1, 5) for j in range(4)]
        t_vals = []
        target_rows = []
        for row in reader:
            if not row:
                continue
            t_vals.append(float(row[col_idx["t"]]))
            target_rows.append([float(row[col_idx[n]]) for n in target_names])

    t_arr = np.asarray(t_vals, dtype=np.float64)
    raw_deg = np.rad2deg(np.asarray(target_rows, dtype=np.float64))
    t_uniform = np.arange(t_arr[0], t_arr[-1], 1.0 / target_hz, dtype=np.float64)
    pchip = interpolate.PchipInterpolator(t_arr, raw_deg, axis=0)
    angles_deg = pchip(t_uniform)
    return target_hz, float(t_uniform[0]), target_names, angles_deg


def detect_hf_segments(
    sig_deg: np.ndarray,
    fs: float,
    t_offset: float,
    cutoff_hz: float = 3.0,
    rms_thresh_deg: float = 0.5,
    min_dur_sec: float = 3.0,
) -> tuple[np.ndarray, list[dict]]:
    """Detects contiguous >= `min_dur_sec` segments where >= `cutoff_hz` 1s RMS >= `rms_thresh_deg`."""
    sos = signal.butter(4, cutoff_hz, btype="highpass", fs=fs, output="sos")
    sig_hp = signal.sosfiltfilt(sos, sig_deg)

    win = max(1, int(round(fs * 1.0)))
    rms_1s = np.sqrt(np.convolve(sig_hp ** 2, np.ones(win) / win, mode="same"))

    above = rms_1s >= rms_thresh_deg
    padded = np.concatenate([[False], above, [False]])
    starts = np.where(~padded[:-1] & padded[1:])[0]
    ends = np.where(padded[:-1] & ~padded[1:])[0]

    min_samples = int(round(min_dur_sec * fs))
    nperseg = max(16, int(round(fs * 2.0)))
    noverlap = int(round(fs * 1.75))
    f_stft, t_stft, Zxx = signal.stft(sig_hp, fs=fs, nperseg=nperseg, noverlap=noverlap, window="hann")
    mag_stft = np.abs(Zxx) * 2.0

    segments = []
    for s_idx, e_idx in zip(starts, ends):
        if (e_idx - s_idx) < min_samples:
            continue
        t_start_rel = s_idx / fs
        t_end_rel = e_idx / fs
        t_mask = (t_stft >= t_start_rel) & (t_stft <= t_end_rel)
        f_mask = f_stft >= cutoff_hz
        sub_spec = mag_stft[np.ix_(f_mask, t_mask)]
        dom_freq = float(f_stft[f_mask][int(np.argmax(np.mean(sub_spec, axis=1)))]) if sub_spec.size > 0 else cutoff_hz
        segments.append(
            {
                "start_s": round(t_offset + t_start_rel, 2),
                "end_s": round(t_offset + t_end_rel, 2),
                "duration_s": round(t_end_rel - t_start_rel, 2),
                "peak_rms_deg": round(float(np.max(rms_1s[s_idx:e_idx])), 3),
                "mean_rms_deg": round(float(np.mean(rms_1s[s_idx:e_idx])), 3),
                "dominant_freq_hz": round(dom_freq, 2),
            }
        )
    return sig_hp, segments


def plot_policy_spectrogram(
    angles_deg: np.ndarray,
    joint_names: list[str],
    fs: float,
    t_offset: float,
    out_path: Path,
    cutoff_hz: float,
    segments_by_joint: dict[str, list[dict]],
    highlight_oscillations: bool,
) -> None:
    """Plots 4x4 velocity spectrograms (0..15 Hz) for 30 Hz policy targets."""
    name_to_idx = {n: i for i, n in enumerate(joint_names)}
    nperseg = max(16, int(round(fs * 2.0)))
    noverlap = int(round(fs * 1.75))

    fig, axes = plt.subplots(4, 4, figsize=(18, 12), sharex=True, sharey=True, constrained_layout=True)
    last_pcm = None
    max_freq_hz = fs / 2.0

    for row_i, (finger, jnames) in enumerate(FINGER_GROUPS):
        for col_j, jname in enumerate(jnames):
            ax = axes[row_i, col_j]
            sig = angles_deg[:, name_to_idx[jname]]
            vel_deg_s = np.gradient(sig, 1.0 / fs)
            f, t_spec, Zxx = signal.stft(vel_deg_s, fs=fs, nperseg=nperseg, noverlap=noverlap, window="hann")
            t_abs = t_spec + t_offset
            amp_db = 20.0 * np.log10(np.abs(Zxx) * 2.0 + 1e-3)

            if highlight_oscillations:
                last_pcm = ax.pcolormesh(
                    t_abs, f, amp_db, shading="gouraud", cmap="turbo", vmin=-10, vmax=28, alpha=0.22
                )
                ax.axhline(cutoff_hz, color="black", linestyle="--", linewidth=2.0, alpha=0.75, zorder=4)
                ax.axhline(cutoff_hz, color="white", linestyle="--", linewidth=1.0, alpha=0.90, zorder=5)
                f_hf_mask = f >= cutoff_hz
                for seg in segments_by_joint[jname]:
                    t_seg_mask = (t_abs >= seg["start_s"]) & (t_abs <= seg["end_s"])
                    if np.any(t_seg_mask) and np.any(f_hf_mask):
                        ax.pcolormesh(
                            t_abs[t_seg_mask],
                            f[f_hf_mask],
                            amp_db[np.ix_(f_hf_mask, t_seg_mask)],
                            shading="gouraud",
                            cmap="turbo",
                            vmin=-10,
                            vmax=28,
                            alpha=1.0,
                            zorder=3,
                        )
                    dur = seg["end_s"] - seg["start_s"]
                    h_box = max_freq_hz - cutoff_hz
                    for ec, lw, zo in [("black", 2.8, 5), ("white", 1.5, 6)]:
                        ax.add_patch(
                            Rectangle(
                                (seg["start_s"], cutoff_hz), dur, h_box,
                                edgecolor=ec, facecolor="none", linewidth=lw, zorder=zo
                            )
                        )
                ax.set_title(f"{jname.replace('target_', '')} (HF>=3s: {len(segments_by_joint[jname])})", fontsize=9)
            else:
                last_pcm = ax.pcolormesh(
                    t_abs, f, amp_db, shading="gouraud", cmap="turbo", vmin=-10, vmax=28
                )
                ax.set_title(jname.replace("target_", ""), fontsize=9)

            ax.set_ylim(0, max_freq_hz)
            if col_j == 0:
                ax.set_ylabel(f"{finger}\nFreq (Hz)", fontsize=9)
            if row_i == 3:
                ax.set_xlabel("Time (s)", fontsize=9)

    mode_label = f"Dashed={cutoff_hz:.1f} Hz, White Box=Oscillation >=3.0s" if highlight_oscillations else "Raw"
    fig.colorbar(last_pcm, ax=axes, label="Velocity Amplitude [dB ref 1 deg/s]", shrink=0.85)
    fig.suptitle(f"30 Hz Policy Target Velocity Spectrograms (STFT 2.0s Hann, {mode_label})", fontsize=12)
    fig.savefig(out_path, dpi=180)
    plt.close(fig)


def main() -> None:
    default_csv = Path(__file__).resolve().parent / "2026-11-18_13-33-46_policy.csv"
    parser = argparse.ArgumentParser(description="Analyze 30 Hz _policy.csv targets.")
    parser.add_argument("--csv", type=Path, default=default_csv)
    parser.add_argument("--cutoff-hz", type=float, default=3.0)
    parser.add_argument("--rms-thresh-deg", type=float, default=0.5)
    parser.add_argument("--min-dur-sec", type=float, default=3.0)
    args = parser.parse_args()

    fs, t_offset, joint_names, angles_deg = load_policy_non_thumb(args.csv)
    out_dir = args.csv.parent

    f_psd, pxx = signal.welch(angles_deg - np.mean(angles_deg, axis=0), fs=fs, nperseg=256, axis=0)
    mean_psd = np.mean(pxx, axis=1)
    band_power = {}
    for lo, hi in [(0, 1), (1, 2), (2, 3), (3, 5), (5, 10), (10, 15)]:
        m = (f_psd >= lo) & (f_psd < hi)
        band_power[f"[{lo}, {hi}) Hz"] = round(float(np.trapezoid(mean_psd[m], f_psd[m])), 4)

    segments_by_joint = {}
    joint_stats = {}
    for i, name in enumerate(joint_names):
        sig_hp, segs = detect_hf_segments(
            angles_deg[:, i], fs=fs, t_offset=t_offset,
            cutoff_hz=args.cutoff_hz, rms_thresh_deg=args.rms_thresh_deg, min_dur_sec=args.min_dur_sec
        )
        segments_by_joint[name] = segs
        joint_stats[name.replace("target_", "")] = {
            "hp_rms_deg": round(float(np.sqrt(np.mean(sig_hp ** 2))), 4),
            "hp_peak_deg": round(float(np.max(np.abs(sig_hp))), 4),
            "segments_ge_3s": segs,
        }

    plot_policy_spectrogram(
        angles_deg, joint_names, fs, t_offset,
        out_dir / "policy_spectrogram_raw_all_non_thumb.png",
        args.cutoff_hz, segments_by_joint, highlight_oscillations=False,
    )
    plot_policy_spectrogram(
        angles_deg, joint_names, fs, t_offset,
        out_dir / "policy_spectrogram_all_non_thumb.png",
        args.cutoff_hz, segments_by_joint, highlight_oscillations=True,
    )

    total_segs = sum(len(v) for v in segments_by_joint.values())
    active_joints = sum(1 for v in segments_by_joint.values() if v)
    summary = {
        "csv_file": args.csv.name,
        "source_hz": int(fs),
        "num_samples": int(angles_deg.shape[0]),
        "t_start_s": round(t_offset, 2),
        "duration_s": round((angles_deg.shape[0] - 1) / fs, 2),
        "band_power_deg2": band_power,
        "total_segments_ge_3s": total_segs,
        "joints_with_segments": active_joints,
        "joints": joint_stats,
    }
    out_json = out_dir / "policy_spectrogram_summary.json"
    out_json.write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
    print(f"Policy 30Hz analysis complete: {active_joints}/16 joints, {total_segs} segments >=3.0s")
    print(json.dumps({"band_power_deg2": band_power, "total_segments_ge_3s": total_segs, "joints_with_segments": active_joints}, indent=2))


if __name__ == "__main__":
    main()
