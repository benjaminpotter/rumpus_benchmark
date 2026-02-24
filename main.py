import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from argparse import ArgumentParser
from pathlib import Path


def main():
    parser = ArgumentParser()
    parser.add_argument("PATH", type=Path)
    args = parser.parse_args()

    df = read_frame_results(args.PATH)
    solution_df = read_results(args.PATH / "results.csv")

    df["candidate_yaw"] = ((df["car_yaw_deg"] + df["yaw_offset_deg"] + 180) % 360) - 180

    # Pick the candidate with the smallest weighted_rmse per frame
    best_candidates = df.loc[df.groupby("frame_index")["weighted_rmse"].idxmin()].copy()
    best_candidates = best_candidates.sort_values("frame_index").reset_index(drop=True)

    # Merge with solution
    merged = best_candidates.merge(
        solution_df[["frame_index", "car_yaw_deg"]],
        on="frame_index",
        suffixes=("_candidate", "_solution"),
    )
    merged["yaw_error"] = merged["candidate_yaw"] - merged["car_yaw_deg_solution"]
    # Wrap error to [-180, 180]
    merged["yaw_error"] = ((merged["yaw_error"] + 180) % 360) - 180

    abs_error = merged["yaw_error"].abs()
    print("=== Yaw Error Summary (Best Candidate vs Solution) ===")
    print(f"  Frames evaluated:     {len(merged)}")
    print(f"  Mean error:           {merged['yaw_error'].mean():.3f}°")
    print(f"  Mean absolute error:  {abs_error.mean():.3f}°")
    print(f"  Median abs error:     {abs_error.median():.3f}°")
    print(f"  Std dev of error:     {merged['yaw_error'].std():.3f}°")
    print(f"  RMSE:                 {np.sqrt((merged['yaw_error'] ** 2).mean()):.3f}°")
    print(
        f"  Max abs error:        {abs_error.max():.3f}° (frame {merged.loc[abs_error.idxmax(), 'frame_index']})"
    )
    print(
        f"  Min abs error:        {abs_error.min():.3f}° (frame {merged.loc[abs_error.idxmin(), 'frame_index']})"
    )
    print(f"  % frames within 0.1°:  {(abs_error <= 0.1).mean() * 100:.1f}%")
    print(f"  % frames within 0.5°:  {(abs_error <= 0.5).mean() * 100:.1f}%")
    print(f"  % frames within 1.0°:  {(abs_error <= 1.0).mean() * 100:.1f}%")
    print()

    # Plot
    fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True)

    axes[0].plot(
        merged["frame_index"],
        merged["candidate_yaw"],
        label="Best Candidate Yaw",
        color="steelblue",
    )
    axes[0].plot(
        merged["frame_index"],
        merged["car_yaw_deg_solution"],
        label="Solution Yaw",
        color="darkorange",
        linestyle="--",
    )
    axes[0].set_ylabel("Yaw (degrees)")
    axes[0].set_title("Best Candidate Yaw vs Solution Yaw")
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(merged["frame_index"], merged["yaw_error"], color="crimson")
    axes[1].axhline(0, color="black", linewidth=0.8, linestyle="--")
    axes[1].set_ylabel("Error (degrees)")
    axes[1].set_xlabel("Frame")
    axes[1].set_title("Yaw Error (Candidate - Solution)")
    axes[1].grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig("yaw_comparison.png", dpi=150)
    plt.show()


def read_frame_results(path):
    frame_results = []
    frame_result_paths = sorted(path.glob("frame_*_results.csv"))

    for path in frame_result_paths:
        df = pd.read_csv(path)
        frame_results.append(df)

    return pd.concat(frame_results, ignore_index=True)


def read_results(path):
    return pd.read_csv(path)


if __name__ == "__main__":
    main()
