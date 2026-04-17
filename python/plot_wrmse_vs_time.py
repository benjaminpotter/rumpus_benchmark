from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib as mpl
import matplotlib.pyplot as plt

FIGURE_PATH = Path("figure")
RESULTS_PATH = Path("benchmarks/bmk4.csv")


def main():
    print("Hello, world!")

    df = read_results(RESULTS_PATH)

    for oi in range(0, 440, 10):
        figname = f"wrmse_over_yaw_fi_125_oi_{oi:03}"
        fig = plot_wrmse_over_yaw(df, fi=125, oi=oi)
        save_figure(fig, figname, ["png"])
        plt.close()

    for oi in range(190, 250, 1):
        figname = f"wrmse_distribution_oi_{oi:03}"
        fig = plot_wrmse_distribution(df, oi=oi)
        save_figure(fig, figname, ["png"])
        plt.close()

    oi = 220
    figname = f"wrmse_distribution_oi_{oi:03}"
    fig = plot_wrmse_distribution(df, oi=oi)
    save_figure(fig, figname, ["pdf"])
    plt.close()

    oi = 199
    figname = f"wrmse_distribution_oi_{oi:03}"
    fig = plot_wrmse_distribution(df, oi=oi)
    save_figure(fig, figname, ["pdf"])
    plt.close()


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


def plot_wrmse_distribution(df, oi=0):
    # 1. Filter for the specific orientation
    df_filtered = df[df["orientation_index"] == oi].copy()

    if df_filtered.empty:
        print(f"No data found for orientation_index {oi}")
        return plt.subplots()[0]

    # --- NEW: Extract pitch and roll for annotation ---
    # Since oi identifies a specific pitch/roll combo, we take the first occurrence
    pitch = df_filtered["cam_pitch_deg"].iloc[0]
    roll = df_filtered["cam_roll_deg"].iloc[0]
    annotation_text = f"Pitch: {pitch:.1f}° | Roll: {roll:.1f}°"

    group = df_filtered.groupby("frame_index")["weighted_rmse"]
    rmin = group.transform("min")
    rmax = group.transform("max")

    # 2. Normalize the WRMSE values
    df_filtered["normalized_wrmse"] = (df_filtered["weighted_rmse"] - rmin) / (rmax - rmin)

    # 3. Pivot the data
    pivot_df = df_filtered.pivot(
        index="yaw_offset_deg", 
        columns="datetime_utc", 
        values="normalized_wrmse"
    )
    pivot_df.columns = pd.to_datetime(pivot_df.columns)
    pivot_df = pivot_df.sort_index(axis=0).sort_index(axis=1)

    # --- NEW: Reindex to expose gaps in time ---
    # We find the smallest time difference to define our "grid step"
    time_deltas = pd.Series(pivot_df.columns).diff().dropna()
    if not time_deltas.empty:
        min_delta = time_deltas.min()
        # Create a full range from start to end using the detected frequency
        full_time_range = pd.date_range(
            start=pivot_df.columns.min(), 
            end=pivot_df.columns.max(), 
            freq=min_delta
        )
        # Reindexing inserts NaN columns for the missing time blocks (e.g., frames 251-999)
        pivot_df = pivot_df.reindex(columns=full_time_range)

    fig, ax = plt.subplots()

    # 4. Use pcolormesh to avoid interpolation
    # shading='auto' handles the coordinates correctly for discrete cells
    mesh = ax.pcolormesh(
        pivot_df.columns, 
        pivot_df.index, 
        pivot_df.values, 
        shading='auto',
        cmap="viridis"
    )

    ax.text(
        0.85, 0.95, 
        annotation_text,
        transform=ax.transAxes,
        horizontalalignment='right',
        verticalalignment='top',
        fontsize=7,
        bbox=dict(boxstyle="round,pad=0.3", facecolor="white", alpha=0.7, edgecolor="none")
    )

    # 5. Aesthetics
    ax.set_xlabel("Time (UTC)")
    ax.set_ylabel("Yaw Offset (deg)")
    plt.xticks(rotation=45)

    cbar = fig.colorbar(mesh)
    cbar.set_label("Normalized Objective\nFunction ($J_{norm}$)")

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
