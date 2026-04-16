import matplotlib.pyplot as plt
import matplotlib as mpl
import numpy as np
import pandas as pd
from argparse import ArgumentParser
from pathlib import Path
from sklearn.linear_model import LinearRegression
from scipy import stats
import plotly.graph_objects as go
from scipy.interpolate import griddata

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
    parser.add_argument("PATH", type=Path)
    args = parser.parse_args()

    df = read_frame_results(args.PATH)
    solution_df = read_results(args.PATH / "results.csv")

    df["candidate_yaw"] = ((df["car_yaw_deg"] + df["yaw_offset_deg"] + 180) % 360) - 180
    df["delta_weighted_rmse"] = df["weighted_rmse"] - df.groupby("frame_index")[
        "weighted_rmse"
    ].transform("min")

    # --- Unwrap candidate_yaw to make it continuous across the -180/180 boundary ---
    # Sort by frame_index so unwrapping is meaningful across time
    df = df.sort_values(["frame_index", "candidate_yaw"]).reset_index(drop=True)

    # Unwrap per frame: within each frame the yaw values are a discrete set,
    # but we need the global yaw axis to be continuous. Unwrap the per-frame
    # yaw range boundaries across frames to detect wrapping.
    # Simpler approach: just unwrap the full candidate_yaw column sorted by value.
    yaw_sorted = np.sort(df["candidate_yaw"].unique())
    yaw_unwrapped = np.unwrap(np.deg2rad(yaw_sorted))  # unwrap in radians
    yaw_unwrapped_deg = np.rad2deg(yaw_unwrapped)

    # Build a mapping from wrapped -> unwrapped yaw
    yaw_map = dict(zip(yaw_sorted, yaw_unwrapped_deg))
    df["candidate_yaw_unwrapped"] = df["candidate_yaw"].map(yaw_map)

    # Also unwrap solution yaw for the scatter overlay
    solution_yaw_rad = np.unwrap(np.deg2rad(solution_df["car_yaw_deg"].values))
    solution_yaw_unwrapped = np.rad2deg(solution_yaw_rad)

    fig = go.Figure()

    yaw_lin = np.linspace(
        df["candidate_yaw_unwrapped"].min(), df["candidate_yaw_unwrapped"].max(), 360
    )
    time_lin = np.linspace(df["frame_index"].min(), df["frame_index"].max(), 360)

    X, Y = np.meshgrid(yaw_lin, time_lin)

    Z = griddata(
        points=(df["candidate_yaw_unwrapped"], df["frame_index"]),
        values=df["delta_weighted_rmse"],
        xi=(X, Y),
        method="linear",
        fill_value=100e-6,
    )

    # --- Mask grid cells that are outside the actual yaw range for each frame ---
    # For each row in the grid (each time_lin value), find the nearest frame's
    # actual yaw extent and mask columns outside it.
    frame_yaw_extent = (
        df.groupby("frame_index")["candidate_yaw_unwrapped"]
        .agg(["min", "max"])
        .reset_index()
    )

    for i, t in enumerate(time_lin):
        # Find the closest frame(s) to this time value
        nearest_idx = (frame_yaw_extent["frame_index"] - t).abs().idxmin()
        yaw_min = frame_yaw_extent.loc[nearest_idx, "min"]
        yaw_max = frame_yaw_extent.loc[nearest_idx, "max"]
        # Mask columns outside the valid yaw range for this time slice
        out_of_range = (yaw_lin < yaw_min) | (yaw_lin > yaw_max)
        Z[i, out_of_range] = np.nan

    fig.add_trace(go.Surface(x=X, y=Y, z=Z, opacity=0.8, hoverinfo="skip"))

    solution_Z = griddata(
        points=(df["candidate_yaw_unwrapped"], df["frame_index"]),
        values=df["delta_weighted_rmse"],
        xi=(solution_yaw_unwrapped, solution_df["frame_index"]),
        method="linear",
    )

    fig.add_trace(
        go.Scatter3d(
            x=solution_yaw_unwrapped,
            y=solution_df["frame_index"],
            z=[0] * len(solution_df),
            mode="lines+markers",
            line=dict(color="red", width=6),
            marker=dict(size=4, color="red"),
            name="Yaw Path",
        )
    )

    fig.update_layout(
        title="Yaw vs Error vs Time",
        scene=dict(
            xaxis_title="Yaw (deg, unwrapped)",
            yaxis_title="Frame Index (Time)",
            zaxis_title="Weighted RMSE",
            aspectratio=dict(x=1, y=10, z=0.5),
        ),
    )

    fig.show()


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
