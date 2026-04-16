#!/usr/bin/env python3
"""Overlay the Hough-estimated solar azimuth on an AoP image."""

import argparse
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path

# Matches the origin used in hough_transform() in the Rust code.
ORIGIN_ROW = 512
ORIGIN_COL = 612


def ray_endpoint(angle_deg: float, origin: tuple, shape: tuple):
    """
    Given an angle (measured as atan2(dy, dx) from the origin, in degrees),
    return the endpoint (col, row) where the ray first exits the image bounds.

    Angle convention matches `angle_of()` in the Rust code:
      0°   → rightward  (+col direction)
      90°  → downward   (+row direction)
    """
    rows, cols = shape[:2]
    ox, oy = origin[1], origin[0]  # (col, row)
    rad = np.deg2rad(angle_deg)
    dx, dy = np.cos(rad), np.sin(rad)

    ts = []
    for t in [
        (0 - ox) / dx if dx != 0 else None,  # left edge
        (cols - 1 - ox) / dx if dx != 0 else None,  # right edge
        (0 - oy) / dy if dy != 0 else None,  # top edge
        (rows - 1 - oy) / dy if dy != 0 else None,  # bottom edge
    ]:
        if t is not None and t > 0:
            ts.append(t)

    t = min(ts)
    return (ox + t * dx, oy + t * dy)  # (col, row)


def plot_overlay(image_path: str, csv_path: str, output_path: str | None = None):
    img = plt.imread(image_path)
    df = pd.read_csv(csv_path)

    best = df.loc[df["votes"].idxmax()]
    angle_deg = best["angle_deg"]

    origin = (ORIGIN_ROW, ORIGIN_COL)
    endpoint = ray_endpoint(-angle_deg, origin, img.shape)

    fig, ax = plt.subplots(figsize=(12, 10))
    ax.imshow(img)
    ax.annotate(
        "",
        xy=endpoint,
        xytext=(ORIGIN_COL, ORIGIN_ROW),
        arrowprops=dict(arrowstyle="-|>", color="red", lw=1.5),
    )
    ax.plot(ORIGIN_COL, ORIGIN_ROW, "r+", markersize=10)

    ax.set_title(f"{Path(image_path).stem} — solar azimuth: {angle_deg:.2f}°")
    ax.axis("off")
    fig.tight_layout()

    if output_path:
        fig.savefig(output_path, dpi=150, bbox_inches="tight")
        print(f"Saved to {output_path}")
    else:
        plt.show()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Overlay Hough-estimated solar azimuth on an AoP image."
    )
    parser.add_argument("image", help="Path to aop_NNNN.png")
    parser.add_argument("csv", help="Path to frame_NNNN_results.csv")
    parser.add_argument(
        "-o", "--output", help="Save plot to this file instead of showing it"
    )
    args = parser.parse_args()

    plot_overlay(args.image, args.csv, args.output)
