from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib as mpl
import matplotlib.pyplot as plt

FIGURE_PATH = Path("figure")
BMK_PATH = Path("benchmarks")


def main():

    run(BMK_PATH / "bmk5.csv", prefix="urban01")
    run(BMK_PATH / "bmk4.csv", prefix="urban04")

def run(file, prefix=""):
    df = read_results(file)

    #for oi in range(0, 440, 10):
    #    figname = f"{prefix}_wrmse_over_yaw_fi_125_oi_{oi:03}"
    #    fig = plot_wrmse_over_yaw(df, fi=125, oi=oi)
    #    save_figure(fig, figname, ["png"])
    #    plt.close()

    #for oi in range(190, 250, 1):
    #    figname = f"{prefix}_wrmse_distribution_oi_{oi:03}"
    #    fig = plot_wrmse_distribution(df, oi=oi)
    #    save_figure(fig, figname, ["png"])
    #    plt.close()

    #oi = 220
    #figname = f"{prefix}_wrmse_distribution_oi_{oi:03}"
    #fig = plot_wrmse_distribution(df, oi=oi)
    #save_figure(fig, figname, ["pdf"])
    #plt.close()

    #oi = 199
    #figname = f"{prefix}_wrmse_distribution_oi_{oi:03}"
    #fig = plot_wrmse_distribution(df, oi=oi)
    #save_figure(fig, figname, ["pdf"])
    #plt.close()

    figname = f"{prefix}_yaw_error_vs_pitch_and_roll"
    fig = plot_yaw_error_vs_pitch_and_roll(df)
    save_figure(fig, figname, ["pdf"])
    plt.close()

    figname = f"{prefix}_wrmse_vs_pitch_and_roll"
    fig = plot_wrmse_vs_pitch_and_roll(df)
    save_figure(fig, figname, ["pdf"])
    plt.close()


def plot_wrmse_vs_pitch_and_roll(df):
    """ Contour plot of the weighted rmse versus the pitch (y axis) and roll (x axis) angles. """

    # 1. Filter for only the rows marked as the 'best' estimate
    df_best = df[df["is_best"]].copy()

    if df_best.empty:
        print("Warning: No data with 'is_best' flag found.")
        return plt.subplots()[0]

    # 2. Aggregate the data for the contour surface
    # We group by orientation to get the average 'best' yaw for that specific coordinate.
    plot_data = df_best.groupby(["cam_pitch_deg", "cam_roll_deg"])["frame_norm_weighted_rmse"].mean().reset_index()

    fig, ax = plt.subplots()

    # 3. Generate the filled contour plot
    levels = np.linspace(0, 1, 15)
    tcf = ax.tricontourf(
        plot_data["cam_roll_deg"], 
        plot_data["cam_pitch_deg"], 
        plot_data["frame_norm_weighted_rmse"], 
        levels=levels, 
        vmin=0,
        vmax=1,
        cmap="RdBu_r"
    )

    # 4. OVERLAY RAW DATA POINTS
    # This plots a small dot at every unique (roll, pitch) coordinate used in the plot.
    ax.scatter(
        plot_data["cam_roll_deg"], 
        plot_data["cam_pitch_deg"], 
        color="black", 
        s=0.5,         # Very small size to avoid clutter
        alpha=0.4,     # Subtle transparency
        marker='.'     # Simple dot marker
    )

    # 5. Aesthetics
    ax.set_xlabel("Roll Angle (deg)")
    ax.set_ylabel("Pitch Angle (deg)")
    
    cbar = fig.colorbar(tcf)
    cbar.set_label("Mean Frame Normalized\nObjective Function")

    return fig


def plot_yaw_error_vs_pitch_and_roll(df):
    """ Contour plot of the yaw error versus the pitch (y axis) and roll (x axis) angles. """

    # 1. Filter for only the rows marked as the 'best' estimate
    df_best = df[df["is_best"]].copy()
    df_best["abs_yaw_error"] = np.abs(df_best["yaw_offset_deg"])

    if df_best.empty:
        print("Warning: No data with 'is_best' flag found.")
        return plt.subplots()[0]

    # 2. Aggregate the data for the contour surface
    # We group by orientation to get the average 'best' yaw for that specific coordinate.
    plot_data = df_best.groupby(["cam_pitch_deg", "cam_roll_deg"])["abs_yaw_error"].mean().reset_index()

    fig, ax = plt.subplots()

    # 3. Generate the filled contour plot
    tcf = ax.tricontourf(
        plot_data["cam_roll_deg"], 
        plot_data["cam_pitch_deg"], 
        plot_data["abs_yaw_error"], 
        levels=15, 
        cmap="RdBu_r"
    )

    # 4. OVERLAY RAW DATA POINTS
    # This plots a small dot at every unique (roll, pitch) coordinate used in the plot.
    ax.scatter(
        plot_data["cam_roll_deg"], 
        plot_data["cam_pitch_deg"], 
        color="black", 
        s=0.5,         # Very small size to avoid clutter
        alpha=0.4,     # Subtle transparency
        marker='.'     # Simple dot marker
    )

    # 5. Aesthetics
    ax.set_xlabel("Roll Angle (deg)")
    ax.set_ylabel("Pitch Angle (deg)")
    
    cbar = fig.colorbar(tcf)
    cbar.set_label("Mean Absolute Yaw Error (deg)")

    return fig


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

    # find the best estimates
    # a best estimate is the lowest weighted_rmse value for each unique frame and orientation index.
    # mark the rows with the best estimate using an 'is_best' flag

    # 1. Calculate the minimum weighted_rmse for each unique (frame_index, orientation_index) pair
    # transform('min') broadcasts the minimum value back to the original dataframe's shape
    min_rmse_per_group = df.groupby(["frame_index", "orientation_index"])["weighted_rmse"].transform("min")

    # 2. Mark the rows where the weighted_rmse matches the minimum of its group
    df["is_best"] = df["weighted_rmse"] == min_rmse_per_group

    min_rmse_per_frame = df.groupby(["frame_index"])["weighted_rmse"].transform("min")
    max_rmse_per_frame = df.groupby(["frame_index"])["weighted_rmse"].transform("max")
    df["frame_norm_weighted_rmse"] = (df["weighted_rmse"] - min_rmse_per_frame) / (max_rmse_per_frame - min_rmse_per_frame)

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
