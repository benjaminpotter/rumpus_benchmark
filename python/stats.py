import matplotlib.pyplot as plt
import matplotlib as mpl
import numpy as np
import pandas as pd
from argparse import ArgumentParser
from pathlib import Path
from sklearn.linear_model import LinearRegression
from scipy import stats

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

"""
How can we measure the effect of the treatment on the WRMSE metric?

I have an independent categorical variable which is treatment (tilt-aware) or no treatment (tilt-agnostic).
I have a dependent continuous variable called the weighted rmse (WRMSE).
I have a confounding variable that is also independent called the zenith angle.
The zenith angle has an impact on the dependent variable.

The treatment corresponds to a change in the algorithm.
In the tilt-agnostic case, the zenith angle is not included in the algorithm.
The treatment algorithm extends the control case to consider the zenith angle in its calculation.

Since I have an algorithm, I can run both the treatment and control on identical inputs.
This makes the test a within-condition comparison on paired data.
I have 100 different input conditions, which correspond to real measured data.
I want to test the hypothesis that the treatment reduces the dependent variable (WRMSE).

"""


def main():
    parser = ArgumentParser()
    parser.add_argument("control", type=Path)
    parser.add_argument("treatment", type=Path)
    args = parser.parse_args()

    df_control = pd.read_csv(args.control)
    df_treatmt = pd.read_csv(args.treatment)

    df = pd.merge(
        df_control,
        df_treatmt[["frame_index", "weighted_rmse"]],
        how="inner",
        on="frame_index",
    )

    df["pitch"] = np.deg2rad(df["car_pitch_deg"])
    df["roll"] = np.deg2rad(df["car_roll_deg"])
    df["zenith_angle"] = np.arccos(np.cos(df["pitch"]) * np.cos(df["roll"]))
    df["zenith_angle_deg"] = np.rad2deg(df["zenith_angle"])

    # Paired t-test (two-sided by default)
    t_stat, p_two_sided = stats.ttest_rel(df["weighted_rmse_x"], df["weighted_rmse_y"])

    # Convert to one-sided test (treatment reduces WRMSE)
    if t_stat > 0:
        p_one_sided = p_two_sided / 2
    else:
        p_one_sided = 1 - (p_two_sided / 2)

    print("---- SIGNIFICANCE")
    print("t statistic:", t_stat)
    print("One-sided p-value:", p_one_sided)

    differences = df["weighted_rmse_x"] - df["weighted_rmse_y"]
    mean_diff = np.mean(differences)
    std_diff = np.std(differences, ddof=1)
    cohens_d = mean_diff / std_diff

    print("---- EFFECT SIZE")
    print("Mean difference:", mean_diff)
    print("Standard deviation of differences:", std_diff)
    print("Cohen's d:", cohens_d)


if __name__ == "__main__":
    main()
