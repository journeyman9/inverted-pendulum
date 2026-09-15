#!/usr/bin/env python3
import sys
import pandas as pd
import matplotlib.pyplot as plt

def plot_replay(csv_path: str):
    df = pd.read_csv(csv_path)

    t = df["timestamp_ms"]

    fig, (ax_theta, ax_omega) = plt.subplots(2, 1, sharex=True, figsize=(10, 6))

    ax_theta.plot(t, df["theta_logged"], label="Logged (firmware)", linewidth=1.5)
    ax_theta.plot(t, df["theta_replayed"], label="Replayed (offline)", linewidth=1.5, linestyle="--")
    ax_theta.set_ylabel("theta (rad)")
    ax_theta.legend()
    ax_theta.grid(True, alpha=0.3)

    ax_omega.plot(t, df["omega_logged"], label="Logged (firmware)", linewidth=1.5)
    ax_omega.plot(t, df["omega_replayed"], label="Replayed (offline)", linewidth=1.5, linestyle="--")
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
