from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib as mpl
import matplotlib.pyplot as plt

FIGURE_PATH = Path("figure")
RESULTS_PATH = Path("benchmarks/bmk3.csv")


def main():
    print("Hello, world!")

    df = read_results(RESULTS_PATH)

    for oi in range(0, 440, 10):
        figname = f"wrmse_over_yaw_fi_125_oi_{oi}"
        fig = plot_wrmse_over_yaw(df, fi=125, oi=oi)
        save_figure(fig, figname, ["png"])
        plt.close()

    figname = "wrmse_distribution"
    fig = plot_wrmse_distribution(df)
    save_figure(fig, figname, ["pdf"])


def plot_wrmse_over_yaw(df, fi=0, oi=0):

    # 1. Filter for the specific orientation
    df_filtered = df[(df["frame_index"] == fi) & (df["orientation_index"] == oi)].copy()

    if df_filtered.empty:
        print(f"No data found for frame_index {fi} and orientation_index {oi}")
        return plt.subplots()[0]

    # 2. Normalize the WRMSE values (0 to 1 scale)
    max_val = df_filtered["weighted_rmse"].max()
    min_val = df_filtered["weighted_rmse"].min()
    df_filtered["normalized_wrmse"] = (df_filtered["weighted_rmse"] - min_val) / (max_val - min_val)

    fig, ax = plt.subplots()

    ax.plot(df_filtered["yaw_offset_deg"], df_filtered["normalized_wrmse"])
    
    ax.set_xlabel("Yaw Offset (deg)")
    ax.set_ylabel("Normalized Objective Function")

    return fig


def plot_wrmse_distribution(df, orientation_index=0):
    """
    Generate a contour plot of normalized weighted_rmse values for datetime_utc (x axis) and yaw_offset_deg (y axis).
    """
    # 1. Filter for the specific orientation
    df_filtered = df[df["orientation_index"] == orientation_index].copy()

    if df_filtered.empty:
        print(f"No data found for orientation_index {orientation_index}")
        return plt.subplots()[0]

    # 2. Normalize the WRMSE values (0 to 1 scale)
    max_val = df_filtered["weighted_rmse"].max()
    min_val = df_filtered["weighted_rmse"].min()
    df_filtered["normalized_wrmse"] = (df_filtered["weighted_rmse"] - min_val) / (max_val - min_val)

    # 3. Pivot the data to create a grid for the contour plot
    # Index = Y-axis (Yaw), Columns = X-axis (Time), Values = Z-axis (RMSE)
    pivot_df = df_filtered.pivot(
        index="yaw_offset_deg", 
        columns="datetime_utc", 
        values="normalized_wrmse"
    )

    # Convert columns to datetime objects if they aren't already to ensure proper spacing
    pivot_df.columns = pd.to_datetime(pivot_df.columns)
    
    # Sort to ensure the plot lines up correctly
    pivot_df = pivot_df.sort_index(axis=0).sort_index(axis=1)

    fig, ax = plt.subplots(figsize=(12, 6))

    # 4. Create the contour plot
    # Using 'levels' to define the granularity of the heatmap/contour
    contour = ax.contourf(
        pivot_df.columns, 
        pivot_df.index, 
        pivot_df.values, 
        levels=20, 
        cmap="viridis"
    )

    # 5. Aesthetics and Labels
    ax.set_title(f"Normalized WRMSE Distribution (Orientation Index: {orientation_index})")
    ax.set_xlabel("Time (UTC)")
    ax.set_ylabel("Yaw Offset (deg)")
    
    # Rotate x-axis dates for better readability
    plt.xticks(rotation=45)

    # Add a colorbar
    cbar = fig.colorbar(contour)
    cbar.set_label("Normalized WRMSE")

    fig.tight_layout()
    return fig


def read_results(path):
    """
    Every row in the dataframe corresponds to a unique frame_index, orientation_index, and candidate_index.
    There is a unique weighted_rmse for each row.

    The frame_index identifies which frame of data we are currently considering.
    This is set by the dataset itself, where each frame is a unique measurement during the road trajectory.
    Frames may be disjoint since we do not consider all of the original measurements due to the computational complexity.
    Instead, we consider ranges of frames and within each range we may consider a step size too.
    For example, we consider frames from 100 to 250 and 1000 to 1125 with a 5 frame step size.
    This means the dataframe has frame indices 100, 105, 110, ... 245, 1000, 1005, ... 1120.
    Each unique frame_index also has a unique datetime_utc associated with the original measurement.

    The orientation_index identifies which pitch and roll combination we are currently considering.

    The candidate_index identifies which yaw we are currently considering.
    """

    df = pd.read_csv(path)
    df = df.dropna()

    return df


def save_figure(fig, name, exts):
    for ext in exts:
        fig.savefig(FIGURE_PATH / f"{ext}/{name}.{ext}")


mpl.rcParams.update(
    {
        # Figure size
        "figure.figsize": (3.5, 2.16),  # width, height in inches
        "figure.dpi": 300,
        "savefig.dpi": 300,
        "savefig.bbox": "tight",
        # Fonts
        "font.family": "serif",
        "font.size": 8,
        "axes.labelsize": 8,
        "axes.titlesize": 8,
        "xtick.labelsize": 7,
        "ytick.labelsize": 7,
        "legend.fontsize": 7,
        # Lines / markers
        "lines.linewidth": 1.0,
        "lines.markersize": 3,
        # Axes
        "axes.linewidth": 0.8,
        "axes.grid": True,
        "grid.linewidth": 0.5,
        "grid.alpha": 0.4,
        # Ticks
        "xtick.direction": "in",
        "ytick.direction": "in",
        "xtick.major.size": 3,
        "ytick.major.size": 3,
        "xtick.minor.size": 1.5,
        "ytick.minor.size": 1.5,
        # Legend
        "legend.frameon": True,
        # Math text
        "mathtext.fontset": "stix",
    }
)


if __name__ == "__main__":
    main()
