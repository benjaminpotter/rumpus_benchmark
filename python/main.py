import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
import numpy as np
import pandas as pd
from argparse import ArgumentParser
from pathlib import Path

RESULTS = Path("results")


def main():
    df = load(RESULTS / "results.csv")

    estimate = df["estimated_solar_azimuth_insenu_deg"]
    negative = estimate < 0
    estimate[negative] = estimate[negative] + 180

    error = estimate - df["solar_azimuth_deg"]

    plt.plot(df["utc_time"], error)
    # plt.plot(df["utc_time"], estimate)
    # plt.plot(df["utc_time"], df["solar_azimuth_deg"])
    plt.show()

    return

    """
    Make a visualization with the solar azimuth shown in the carxyz frame and the insenu frame.

    The solar azimuth is defined in two different reference frames.
    The carxyz frame (estimated_solar_azimuth_carxyz_deg) is taken CCW from the X axis.
    The insenu frame (solar_azimuth_deg) is taken CW from the Y (north) axis.
    The carxyz frame and insenu frame are related by the car's yaw angle (car_yaw_deg).
    This angle defines the rotation between the carxyz and insenu frames.
    The car's yaw angle is interpreted as the CCW rotation of the carxyz frame to get the insenu frame.

    I want to show the insenu frame as axes aligned in the visualization.
    Then, the carxyz axes should be shown rotated from the insenu axes using the car's yaw.
    The solar azimuth should be shown using the angles defined in both frames.
    In theory, they should line up perfectly, however this is what I am looking to test.

    I want a slider below the plot that lets me seek through the time axes (utc_time).

    TO ADD:

    I want another set of axes to the right of the existing plot that shows the aop_XXXX.png sidecar file.
    Overtop of this image, I want to show the estimated solar azimuth in the camxyz frame (estimated_solar_azimuth_camxyz_deg).
    In this frame, the solar azimuth angle is taken CCW from the X axis.
    The X axis points towards the right side of the image.
    The Y axis points towards the top of the image.
    The origin is at the center of the image.
    """
    fig, (ax, ax2) = plt.subplots(1, 2, figsize=(15, 7))
    plt.subplots_adjust(bottom=0.2, wspace=0.3)

    def update(val):
        idx = int(val)
        row = df.iloc[idx]

        # --- 1. LEFT PLOT: INS ENU and Car Frame ---
        ax.clear()

        # Plot ENU Frame (Fixed Axes)
        ax.quiver(0, 0, 1, 0, color="black", alpha=0.2, scale=3, label="ENU East")
        ax.quiver(0, 0, 0, 1, color="black", alpha=0.2, scale=3, label="ENU North")

        # Plot Car Frame (Rotated from ENU by Yaw)
        yaw_rad = np.deg2rad(row["car_yaw_deg"])
        car_x = np.cos(-yaw_rad)
        car_y = np.sin(-yaw_rad)
        car_y_x = -np.sin(-yaw_rad)
        car_y_y = np.cos(-yaw_rad)

        ax.quiver(
            0,
            0,
            car_x,
            car_y,
            color="blue",
            scale=3,
            width=0.01,
            label="Car X (Forward)",
        )
        ax.quiver(
            0,
            0,
            car_y_x,
            car_y_y,
            color="cyan",
            scale=3,
            width=0.005,
            label="Car Y (Left)",
        )

        # Solar Azimuth in insenu (CW from North)
        az_enu_rad = np.deg2rad(row["solar_azimuth_deg"])
        sx_enu = np.sin(az_enu_rad)
        sy_enu = np.cos(az_enu_rad)
        ax.quiver(
            -sx_enu,
            -sy_enu,
            sx_enu,
            sy_enu,
            color="orange",
            scale=2,
            width=0.02,
            label="Sun (INS ENU)",
        )

        # Solar Azimuth in carxyz (CCW from Car X)
        az_car_rad = np.deg2rad(row["estimated_solar_azimuth_carxyz_deg"])
        total_angle_rad = -yaw_rad + az_car_rad
        sx_car = np.cos(total_angle_rad)
        sy_car = np.sin(total_angle_rad)
        ax.quiver(
            -sx_car,
            -sy_car,
            sx_car,
            sy_car,
            color="red",
            scale=2,
            width=0.01,
            label="Sun (Est. CarXYZ)",
        )

        az_car_rad = np.deg2rad(row["estimated_solar_azimuth_insenu_deg"])
        total_angle_rad = np.pi / 2 - az_car_rad
        sx_car = np.cos(total_angle_rad)
        sy_car = np.sin(total_angle_rad)
        ax.quiver(
            -sx_car,
            -sy_car,
            sx_car,
            sy_car,
            scale=2,
            width=0.01,
            label="Sun (Est. INS ENU)",
        )

        ax.set_xlim(-1.5, 1.5)
        ax.set_ylim(-1.5, 1.5)
        ax.set_aspect("equal")
        ax.set_title(f"Solar Azimuth Alignment Check\nTime: {row['utc_time']}")
        ax.legend(loc="upper right", fontsize="small")
        ax.grid(True, linestyle=":", alpha=0.6)

        # --- 2. RIGHT PLOT: Camera Image and CamXYZ Azimuth ---
        ax2.clear()

        # Load image based on frame_index (padded to 4 digits)
        frame_idx = int(row["frame_index"])
        img_filename = f"aop_{frame_idx:04d}.png"
        img_path = RESULTS / img_filename

        if img_path.exists():
            img = plt.imread(img_path)
            h, w = img.shape[:2]

            # Show image with origin at center:
            # X right, Y up (standard Cartesian overlay on image)
            # Extent: [left, right, bottom, top]
            ax2.imshow(img, extent=[-w / 2, w / 2, -h / 2, h / 2])

            # Solar Azimuth in camxyz (CCW from Cam X)
            az_cam_rad = np.deg2rad(row["estimated_solar_azimuth_camxyz_deg"])
            vx = np.cos(az_cam_rad)
            vy = np.sin(az_cam_rad)

            # Draw vector starting from center (0,0)
            # 'scale=5' makes the arrow length relative to plot width
            ax2.quiver(
                0,
                0,
                vx,
                vy,
                color="yellow",
                scale=5,
                width=0.01,
                label="Sun (Est. CamXYZ)",
            )

            ax2.set_title(f"Camera View: {img_filename}")
            ax2.set_xlabel("Cam X (Right)")
            ax2.set_ylabel("Cam Y (Up)")
            ax2.legend(loc="upper right", fontsize="small")
        else:
            ax2.text(
                0.5,
                0.5,
                f"Image Not Found:\n{img_filename}",
                ha="center",
                va="center",
                transform=ax2.transAxes,
                color="red",
            )
            ax2.set_title("Camera View (Missing)")

        fig.canvas.draw_idle()

    # Create the slider for time-seeking
    ax_slider = plt.axes([0.2, 0.05, 0.6, 0.03])
    slider = Slider(ax_slider, "Time/Frame", 0, len(df) - 1, valinit=0, valfmt="%d")
    slider.on_changed(update)

    # Initial draw
    update(0)
    plt.show()


def cw_y_to_ccw_x(angle):
    return (angle - 90) % 360


def load(path):
    df = pd.read_csv(path)
    df["utc_time"] = pd.to_datetime(df["utc_time"])

    # solar azimuth deg is defined in the ins enu frame ( clockwise from north )
    # estimated_solar_azimuth_insenu_deg is also clockwise from north
    # car_yaw_deg is ccw from east

    # each row of the dataframe has a unique frame_index
    # there are a few sidecar files that use the frame_index to identify which row they map to
    # the XXXX indicates the frame_index (its always 4 digits padded with leading zeros)
    # side car files:
    # - accumulator_XXXX.csv
    # - aop_XXXX.png
    # - binary_aop_XXXX.png

    return df


if __name__ == "__main__":
    main()
