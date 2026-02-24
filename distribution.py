import matplotlib.pyplot as plt
import matplotlib as mpl
import numpy as np
import pandas as pd
from argparse import ArgumentParser
from pathlib import Path
from sklearn.linear_model import LinearRegression

mpl.rcParams.update(
    {
        # Font settings
        "font.family": "serif",
        # "font.serif": ["Times New Roman", "Times", "Computer Modern Roman"],
        "mathtext.fontset": "cm",
        # Font sizes scaled for 10pt LaTeX document
        "font.size": 8,  # base font size
        "axes.labelsize": 8,
        "axes.titlesize": 8,
        "xtick.labelsize": 6,
        "ytick.labelsize": 6,
        "legend.fontsize": 6,
        # Figure aesthetics
        "axes.linewidth": 0.8,
        "lines.linewidth": 1.0,
        "lines.markersize": 0.5,
        "xtick.major.width": 0.8,
        "ytick.major.width": 0.8,
        # Save figures tightly
        "figure.dpi": 300,
        "savefig.bbox": "tight",
        "savefig.pad_inches": 0.02,
    }
)


def main():
    parser = ArgumentParser()
    parser.add_argument("csvs", nargs="+", type=Path)
    args = parser.parse_args()

    fig, ax = plt.subplots(figsize=(3.3, 2.5))

    for path in args.csvs:
        df = pd.read_csv(path)

        df["pitch"] = np.deg2rad(df["car_pitch_deg"])
        df["roll"] = np.deg2rad(df["car_roll_deg"])

        df["zenith_angle"] = np.arccos(np.cos(df["pitch"]) * np.cos(df["roll"]))
        df["zenith_angle_deg"] = np.rad2deg(df["zenith_angle"])

        append_distribution(ax, df["weighted_rmse"], bins=10, density=False)

    ax.set_xlabel("Weighted RMSE [deg]")
    ax.set_ylabel("Number of Images")
    ax.grid()

    fig.savefig("wrmse_distribution.svg")

    fig, ax = plt.subplots(figsize=(3.3, 2.5))

    for path in args.csvs:
        df = pd.read_csv(path)

        df["pitch"] = np.deg2rad(df["car_pitch_deg"])
        df["roll"] = np.deg2rad(df["car_roll_deg"])

        df["zenith_angle"] = np.arccos(np.cos(df["pitch"]) * np.cos(df["roll"]))
        df["zenith_angle_deg"] = np.rad2deg(df["zenith_angle"])

        append_distribution(ax, df["zenith_angle_deg"], bins=10, density=False)

    ax.set_xlabel("Zenith Angle [deg]")
    ax.set_ylabel("Number of Images")
    ax.grid()

    fig.savefig("zenith_angle_distribution.svg")

    fig, ax = plt.subplots(figsize=(3.3, 2.5))

    df = pd.merge(
        pd.read_csv(args.csvs[0]),
        pd.read_csv(args.csvs[1])[["frame_index", "weighted_rmse"]],
        how="inner",
        on="frame_index",
    )

    delta_wrmse = df["weighted_rmse_x"] - df["weighted_rmse_y"]
    append_distribution(ax, delta_wrmse, bins=10, density=False)

    ax.set_xlabel("$\\Delta$ Weighted RMSE [deg]")
    ax.set_ylabel("Number of Images")
    ax.grid()

    fig.savefig("delta_wrmse_distribution.svg")


def _compute_histogram_line(data, bins=50, density=True, range=None):
    """
    Compute histogram and return bin centers + values
    suitable for line plotting.
    """
    counts, edges = np.histogram(data, bins=bins, density=density, range=range)
    centers = (edges[:-1] + edges[1:]) / 2
    return centers, counts


def plot_distribution(data, bins=50, density=True, ax=None, label=None, **plot_kwargs):
    """
    Create a histogram-as-line distribution plot.

    Returns the matplotlib axes object so more
    datasets can be appended later.
    """
    if ax is None:
        fig, ax = plt.subplots()

    x, y = _compute_histogram_line(data, bins=bins, density=density)
    ax.plot(x, y, label=label, **plot_kwargs)

    ax.set_ylabel("Density" if density else "Count")
    ax.set_xlabel("Value")

    if label is not None:
        ax.legend()

    return ax


def append_distribution(ax, data, bins=50, density=True, label=None, **plot_kwargs):
    """
    Append a new dataset to an existing axes.
    """
    x, y = _compute_histogram_line(data, bins=bins, density=density)
    ax.plot(x, y, label=label, **plot_kwargs)

    if label is not None:
        ax.legend()

    return ax


if __name__ == "__main__":
    main()
