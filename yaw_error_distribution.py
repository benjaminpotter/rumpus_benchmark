import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from argparse import ArgumentParser
from pathlib import Path


def main():
    parser = ArgumentParser(
        description="Plot the distribution of yaw errors (best candidate vs solution)."
    )
    parser.add_argument("PATH", type=Path)
    args = parser.parse_args()

    df = read_frame_results(args.PATH)
    solution_df = pd.read_csv(args.PATH / "results.csv")

    df["candidate_yaw"] = ((df["car_yaw_deg"] + df["yaw_offset_deg"] + 180) % 360) - 180

    # Pick the candidate with the smallest weighted_rmse per frame
    best_candidates = df.loc[df.groupby("frame_index")["weighted_rmse"].idxmin()].copy()
    best_candidates = best_candidates.sort_values("frame_index").reset_index(drop=True)

    # Merge with solution and compute wrapped yaw error
    merged = best_candidates.merge(
        solution_df[["frame_index", "car_yaw_deg"]],
        on="frame_index",
        suffixes=("_candidate", "_solution"),
    )
    merged["yaw_error"] = merged["candidate_yaw"] - merged["car_yaw_deg_solution"]
    merged["yaw_error"] = ((merged["yaw_error"] + 180) % 360) - 180

    errors = merged["yaw_error"]
    abs_errors = errors.abs()
    mean_err = errors.mean()
    std_err = errors.std()

    fig, axes = plt.subplots(1, 2, figsize=(14, 5))

    # --- Signed error histogram ---
    ax = axes[0]
    ax.hist(errors, bins=50, color="steelblue", edgecolor="white", linewidth=0.5)
    ax.axvline(
        mean_err,
        color="darkorange",
        linewidth=1.5,
        linestyle="--",
        label=f"Mean: {mean_err:.2f}°",
    )
    ax.axvline(
        mean_err + std_err,
        color="darkorange",
        linewidth=1,
        linestyle=":",
        label=f"±1 std: {std_err:.2f}°",
    )
    ax.axvline(mean_err - std_err, color="darkorange", linewidth=1, linestyle=":")
    ax.axvline(0, color="black", linewidth=0.8)
    ax.set_xlabel("Yaw Error (degrees)")
    ax.set_ylabel("Count")
    ax.set_title("Signed Yaw Error Distribution")
    ax.legend()
    ax.grid(True, alpha=0.3)

    # --- Absolute error CDF ---
    ax = axes[1]
    sorted_abs = np.sort(abs_errors)
    cdf = np.arange(1, len(sorted_abs) + 1) / len(sorted_abs)
    ax.plot(sorted_abs, cdf * 100, color="steelblue", linewidth=2)
    for threshold, style in [(0.1, "--"), (0.5, ":"), (1.0, "-.")]:
        pct = (abs_errors <= threshold).mean() * 100
        ax.axvline(
            threshold,
            color="crimson",
            linewidth=1,
            linestyle=style,
            label=f"≤{threshold}°: {pct:.1f}%",
        )
    ax.set_xlabel("Absolute Yaw Error (degrees)")
    ax.set_ylabel("Cumulative % of Frames")
    ax.set_title("Absolute Yaw Error CDF")
    ax.legend()
    ax.grid(True, alpha=0.3)

    plt.suptitle(
        "Yaw Error Distribution — Best Candidate vs Solution",
        fontsize=13,
        fontweight="bold",
    )
    plt.tight_layout()
    plt.savefig("yaw_error_distribution.png", dpi=150)
    plt.show()


def read_frame_results(path):
    frame_results = []
    for p in sorted(path.glob("frame_*_results.csv")):
        frame_results.append(pd.read_csv(p))
    return pd.concat(frame_results, ignore_index=True)


if __name__ == "__main__":
    main()
