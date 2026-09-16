#!/usr/bin/env python3
import sys
import pandas as pd
import matplotlib.pyplot as plt

def plot_replay(csv_path: str):
    df = pd.read_csv(csv_path)

    t = df["timestamp_ms"]

    has_reseed = "reseed" in df.columns

    fig, (ax_theta, ax_omega) = plt.subplots(2, 1, sharex=True, figsize=(10, 6))

    if has_reseed:
        reseed = df["reseed"].values
        gap_indices = list(df.index[reseed == 1])

        # Plot each contiguous segment separately so no line crosses a gap
        seg_starts = gap_indices  # each reseed starts a new segment
        for seg_idx, start in enumerate(seg_starts):
            end = seg_starts[seg_idx + 1] if seg_idx + 1 < len(seg_starts) else len(df)
            seg = df.iloc[start:end]
            ts = seg["timestamp_ms"]
            label_l = "Logged (firmware)" if seg_idx == 0 else None
            label_r = "Replayed (offline)" if seg_idx == 0 else None
            ax_theta.plot(ts, seg["theta_logged"], color="#1f77b4", linewidth=1.5, label=label_l)
            ax_theta.plot(ts, seg["theta_replayed"], color="#ff7f0e", linewidth=1.5, linestyle="--", label=label_r)
            ax_omega.plot(ts, seg["omega_logged"], color="#1f77b4", linewidth=1.5, label=label_l)
            ax_omega.plot(ts, seg["omega_replayed"], color="#ff7f0e", linewidth=1.5, linestyle="--", label=label_r)

        # Red dots at reseed points
        gap = df[reseed == 1]
        gt = gap["timestamp_ms"]
        ax_theta.scatter(gt, gap["theta_logged"], color="red", s=50, zorder=5, label="Buffer gap (reseed)")
        ax_theta.scatter(gt, gap["theta_replayed"], color="red", s=50, zorder=5, marker="x")
        ax_omega.scatter(gt, gap["omega_logged"], color="red", s=50, zorder=5, label="Buffer gap (reseed)")
        ax_omega.scatter(gt, gap["omega_replayed"], color="red", s=50, zorder=5, marker="x")
    else:
        ax_theta.plot(t, df["theta_logged"], color="#1f77b4", linewidth=1.5, label="Logged (firmware)")
        ax_theta.plot(t, df["theta_replayed"], color="#ff7f0e", linewidth=1.5, linestyle="--", label="Replayed (offline)")
        ax_omega.plot(t, df["omega_logged"], color="#1f77b4", linewidth=1.5, label="Logged (firmware)")
        ax_omega.plot(t, df["omega_replayed"], color="#ff7f0e", linewidth=1.5, linestyle="--", label="Replayed (offline)")

    ax_theta.set_ylabel("theta (rad)")
    ax_theta.legend()
    ax_theta.grid(True, alpha=0.3)

    ax_omega.set_ylabel("omega (rad/s)")
    ax_omega.set_xlabel("timestamp (ms)")
    ax_omega.legend()
    ax_omega.grid(True, alpha=0.3)

    fig.suptitle(csv_path.split("/")[-1])
    fig.tight_layout()

    out_path = csv_path.replace(".csv", ".png")
    fig.savefig(out_path, dpi=150)
    print(f"Saved plot to {out_path}")
    plt.show()

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python plot_replay.py replay_data.csv [replay_data2.csv ...]")
        sys.exit(1)
    for path in sys.argv[1:]:
        plot_replay(path)
