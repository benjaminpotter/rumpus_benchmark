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

    frame_indices = sorted(df["frame_index"].unique())

    # Build one frame per frame_index
    frames = []
    for fi in frame_indices:
        frame_df = df[df["frame_index"] == fi].sort_values("candidate_yaw")
        sol_row = solution_df[solution_df["frame_index"] == fi]

        traces = [
            go.Scatter(
                x=frame_df["candidate_yaw"],
                y=frame_df["delta_weighted_rmse"],
                mode="lines+markers",
                name="Error curve",
                line=dict(color="steelblue"),
                marker=dict(size=3),
            )
        ]

        # Overlay the solution yaw as a vertical line if present
        if not sol_row.empty:
            sol_yaw = sol_row["car_yaw_deg"].values[0]
            traces.append(
                go.Scatter(
                    x=[sol_yaw, sol_yaw],
                    y=[0, frame_df["delta_weighted_rmse"].max()],
                    mode="lines",
                    name="Solution yaw",
                    line=dict(color="red", width=2, dash="dash"),
                )
            )

        frames.append(go.Frame(data=traces, name=str(fi)))

    # Initial frame data
    init_df = df[df["frame_index"] == frame_indices[0]].sort_values("candidate_yaw")
    init_sol = solution_df[solution_df["frame_index"] == frame_indices[0]]

    init_traces = [
        go.Scatter(
            x=init_df["candidate_yaw"],
            y=init_df["delta_weighted_rmse"],
            mode="lines+markers",
            name="Error curve",
            line=dict(color="steelblue"),
            marker=dict(size=3),
        )
    ]
    if not init_sol.empty:
        sol_yaw = init_sol["car_yaw_deg"].values[0]
        init_traces.append(
            go.Scatter(
                x=[sol_yaw, sol_yaw],
                y=[0, init_df["delta_weighted_rmse"].max()],
                mode="lines",
                name="Solution yaw",
                line=dict(color="red", width=2, dash="dash"),
            )
        )

    fig = go.Figure(data=init_traces, frames=frames)

    # Slider steps — one per frame
    sliders = [
        dict(
            active=0,
            currentvalue=dict(prefix="Frame: ", visible=True, xanchor="center"),
            pad=dict(t=50),
            steps=[
                dict(
                    method="animate",
                    args=[
                        [str(fi)],
                        dict(
                            mode="immediate",
                            frame=dict(duration=0, redraw=True),
                            transition=dict(duration=0),
                        ),
                    ],
                    label=str(fi),
                )
                for fi in frame_indices
            ],
        )
    ]

    y_max = df["delta_weighted_rmse"].max()
    fig.update_layout(
        title="Yaw vs Delta RMSE",
        xaxis_title="Candidate Yaw (deg)",
        yaxis_title="Delta Weighted RMSE",
        xaxis=dict(range=[-180, 180]),
        yaxis=dict(range=[0, y_max * 1.05]),
        sliders=sliders,
        updatemenus=[
            dict(
                type="buttons",
                showactive=False,
                y=0,
                x=0.5,
                xanchor="center",
                yanchor="top",
                pad=dict(t=60),
                buttons=[
                    dict(
                        label="Play",
                        method="animate",
                        args=[
                            None,
                            dict(
                                frame=dict(duration=100, redraw=True),
                                fromcurrent=True,
                                transition=dict(duration=0),
                            ),
                        ],
                    ),
                    dict(
                        label="Pause",
                        method="animate",
                        args=[
                            [None],
                            dict(
                                mode="immediate",
                                frame=dict(duration=0, redraw=True),
                                transition=dict(duration=0),
                            ),
                        ],
                    ),
                ],
            )
        ],
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
