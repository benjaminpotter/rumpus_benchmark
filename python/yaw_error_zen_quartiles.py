import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from argparse import ArgumentParser
from pathlib import Path


def main():
    parser = ArgumentParser(
        description="Plot absolute yaw error CDF segmented by zenith_angle_deg quartile."
    )
    parser.add_argument("PATH", type=Path)
    args = parser.parse_args()

    df = read_frame_results(args.PATH)
    solution_df = pd.read_csv(args.PATH / "results.csv")
    solution_df["pitch"] = np.deg2rad(solution_df["car_pitch_deg"])
    solution_df["roll"] = np.deg2rad(solution_df["car_roll_deg"])

    solution_df["zenith_angle"] = np.arccos(
        np.cos(solution_df["pitch"]) * np.cos(solution_df["roll"])
    )
    solution_df["zenith_angle_deg"] = np.rad2deg(solution_df["zenith_angle"])

    df["candidate_yaw"] = ((df["car_yaw_deg"] + df["yaw_offset_deg"] + 180) % 360) - 180

    # Pick the candidate with the smallest weighted_rmse per frame
    best_candidates = df.loc[df.groupby("frame_index")["weighted_rmse"].idxmin()].copy()
    best_candidates = best_candidates.sort_values("frame_index").reset_index(drop=True)

    # Merge with solution
    merged = best_candidates.merge(
        solution_df[["frame_index", "car_yaw_deg", "zenith_angle_deg"]],
        on="frame_index",
        suffixes=("_candidate", "_solution"),
    )
    merged["yaw_error"] = merged["candidate_yaw"] - merged["car_yaw_deg_solution"]
    merged["yaw_error"] = ((merged["yaw_error"] + 180) % 360) - 180
    merged["abs_yaw_error"] = merged["yaw_error"].abs()

    # Assign zenith quartiles
    merged["zenith_quartile"] = pd.qcut(merged["zenith_angle_deg"], q=4)
    quartiles = merged["zenith_quartile"].cat.categories

    colors = ["steelblue", "darkorange", "seagreen", "crimson"]

    fig, ax = plt.subplots(figsize=(9, 6))

    for quartile, color in zip(quartiles, colors):
        subset = merged[merged["zenith_quartile"] == quartile]["abs_yaw_error"]
        sorted_err = np.sort(subset)
        cdf = np.arange(1, len(sorted_err) + 1) / len(sorted_err)
        label = f"{quartile} (n={len(subset)})"
        ax.plot(sorted_err, cdf * 100, color=color, linewidth=2, label=label)

    for threshold, style in [(0.1, "--"), (0.5, ":"), (1.0, "-.")]:
        ax.axvline(
            threshold,
            color="grey",
            linewidth=0.8,
            linestyle=style,
            label=f"{threshold}° threshold",
        )

    ax.set_xlabel("Absolute Yaw Error (degrees)")
    ax.set_ylabel("Cumulative % of Frames")
    ax.set_title("Absolute Yaw Error CDF by Zenith Angle Quartile")
    ax.legend(title="Zenith Angle (deg)", fontsize=9)
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig("yaw_error_cdf_by_zenith.png", dpi=150)
    plt.show()


def read_frame_results(path):
    frame_results = []
    for p in sorted(path.glob("frame_*_results.csv")):
        frame_results.append(pd.read_csv(p))
    return pd.concat(frame_results, ignore_index=True)


if __name__ == "__main__":
    main()
