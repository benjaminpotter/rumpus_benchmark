#!/usr/bin/env python3
"""Plot Hough transform accumulator results from a frame CSV."""

import argparse
import pandas as pd
import matplotlib.pyplot as plt


def plot_accumulator(csv_path: str, output_path: str | None = None):
    df = pd.read_csv(csv_path)

    best = df.loc[df["votes"].idxmax()]

    fig, ax = plt.subplots(figsize=(12, 4))
    ax.plot(df["angle_deg"], df["votes"], linewidth=0.8, color="steelblue")
    ax.axvline(
        best["angle_deg"],
        color="tomato",
        linestyle="--",
        linewidth=1.2,
        label=f"peak: {best['angle_deg']:.2f}°",
    )

    ax.set_xlabel("Angle (degrees)")
    ax.set_ylabel("Votes")
    ax.set_title(f"Hough Accumulator — {csv_path}")
    ax.set_xlim(0, 180)
    ax.legend()
    ax.grid(True, alpha=0.3)
    fig.tight_layout()

    if output_path:
        fig.savefig(output_path, dpi=150)
        print(f"Saved to {output_path}")
    else:
        plt.show()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Plot a Hough accumulator CSV.")
    parser.add_argument("csv", help="Path to frame_NNNN_results.csv")
    parser.add_argument(
        "-o", "--output", help="Save plot to this file instead of showing it"
    )
    args = parser.parse_args()

    plot_accumulator(args.csv, args.output)
