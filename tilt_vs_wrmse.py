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
        plot_bmk(df, ax)

    ax.set_xlabel("Zenith Angle [deg]")
    ax.set_ylabel("Weighted RMSE [deg]")
    ax.grid()

    fig.savefig("tilt_vs_wrmse.svg")


def plot_bmk(df, ax):
    df["pitch"] = np.deg2rad(df["car_pitch_deg"])
    df["roll"] = np.deg2rad(df["car_roll_deg"])

    df["zenith_angle"] = np.arccos(np.cos(df["pitch"]) * np.cos(df["roll"]))
    df["zenith_angle_deg"] = np.rad2deg(df["zenith_angle"])

    X = df[["zenith_angle_deg"]]
    y = df["weighted_rmse"]

    model = LinearRegression()
    model.fit(X, y)

    df["weighted_rmse_pred"] = model.predict(X)

    ax.scatter(df["zenith_angle_deg"], df["weighted_rmse"], label="Image")
    ax.plot(df["zenith_angle_deg"], df["weighted_rmse_pred"])


if __name__ == "__main__":
    main()
