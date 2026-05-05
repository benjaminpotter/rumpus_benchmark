import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
import pandas as pd
import numpy as np
from scipy.optimize import curve_fit


def main():

    df = load()

    p_wrmse_vs_yaw(df)

    model = fit(df)
    p_wrmse_vs_tilt(df, model)


def fit(df):
    """
    Fits a model where WRMSE is a function of yaw:
    WRMSE = A(p, r) * sin(f(p, r) * yaw + phase) + Offset(p, r)

    Referencing plot_tilt_effect.py [1] logic for data columns.
    """

    # 1. Define the complex model
    def hierarchical_sinusoid(
        coords,
        ap,
        ar,
        a0,  # Amplitude coefficients
        op,
        orr,
        o0,  # Offset (DC) coefficients
        pp,
        pr,
        p0,
    ):

        pitch, roll, yaw = coords

        # Calculate modulating parameters based on pitch and roll
        amplitude = ap * pitch + ar * roll + a0
        frequency = 2  # there should be 2 cycles per 360 yaw degrees
        phase = pp * pitch + pr * roll + p0
        dc_offset = op * pitch + orr * roll + o0

        # Convert yaw to radians and apply the sinusoid
        # Standard yaw frequency is usually 1 (one cycle per 360 degrees)
        return amplitude * np.sin(frequency * yaw + phase) + dc_offset

    # 2. Extract input/output data
    x_data = (
        df["cam_pitch_rad"].values,
        df["cam_roll_rad"].values,
        df["car_yaw_rad"].values,
    )
    y_data = df["weighted_rmse"].values

    # 3. Initial Guesses
    # [ap, ar, a0, op, orr, o0, phase]
    # We assume a base amplitude of 1, base frequency of 1,
    # and use the mean wrmse as the base offset.
    initial_guess = [
        0.1,
        0.1,
        1.0,
        0.1,
        0.1,
        0.0,  # np.mean(y_data),
        0.0,
        0.0,
        0.0,
    ]

    try:
        popt, _ = curve_fit(hierarchical_sinusoid, x_data, y_data, p0=initial_guess)

        print("--- Model Parameters Found ---")
        print(f"Amplitude = ({popt[0]:.3f}*P + {popt[1]:.3f}*R + {popt[2]:.3f})")
        print(f"DC Offset = ({popt[3]:.3f}*P + {popt[4]:.3f}*R + {popt[5]:.3f})")
        print(f"Phase     = ({popt[6]:.3f}*P + {popt[7]:.3f}*R + {popt[8]:.3f})")

        return popt
    except Exception as e:
        print(f"Hierarchical fit failed: {e}")
        return None


def p_wrmse_vs_tilt(df, model):
    """
    Show an interactive plot of the weighted_rmse vs car_yaw with user
    configurable sliders for cam_pitch and cam_roll.
    """
    if model is None:
        print("Model fitting failed. Cannot plot.")
        return

    # Unpack model parameters
    ap, ar, a0, op, orr, o0, pp, pr, p0 = model

    # Helper function for model prediction
    def predict_wrmse(pitch, roll, yaw):

        # Calculate modulating parameters based on pitch and roll
        amplitude = ap * pitch + ar * roll + a0
        frequency = 2
        phase = pp * pitch + pr * roll + p0
        dc_offset = op * pitch + orr * roll + o0

        # Standard yaw frequency is usually 1 (one cycle per 360 degrees)
        yaw = np.deg2rad(yaw)
        return amplitude * np.sin(frequency * yaw + phase) + dc_offset

    # 1. Setup the figure and axis
    fig, ax = plt.subplots(figsize=(10, 6))
    plt.subplots_adjust(bottom=0.25)  # Make room for sliders

    # Generate a smooth range for the yaw (X-axis)
    yaw_range = np.linspace(df["car_yaw_deg"].min(), df["car_yaw_deg"].max(), 100)

    # Initial slider values (center of the data range)
    init_pitch = df["cam_pitch_deg"].mean()
    init_roll = df["cam_roll_deg"].mean()

    # Create the plot lines
    # Predicted line (Smooth curve)
    (pred_line,) = ax.plot(
        yaw_range,
        predict_wrmse(init_pitch, init_roll, yaw_range),
        "r-",
        label="Model Prediction",
        alpha=0.8,
    )

    # Empirical scatter (Nearest data points)
    (emp_dots,) = ax.plot([], [], "bo", label="Nearest Empirical Data", markersize=4)

    ax.set_xlabel("Car Yaw [deg]")
    ax.set_ylabel("WRMSE [deg]")
    ax.set_ylim(df["weighted_rmse"].min() * 0.9, df["weighted_rmse"].max() * 1.1)
    ax.legend(loc="upper right")
    ax.grid(True, linestyle="--", alpha=0.6)

    # 2. Add Sliders
    ax_pitch = plt.axes([0.2, 0.1, 0.6, 0.03])
    ax_roll = plt.axes([0.2, 0.05, 0.6, 0.03])

    s_pitch = Slider(
        ax_pitch,
        "Cam Pitch",
        df["cam_pitch_deg"].min(),
        df["cam_pitch_deg"].max(),
        valinit=init_pitch,
    )
    s_roll = Slider(
        ax_roll,
        "Cam Roll",
        df["cam_roll_deg"].min(),
        df["cam_roll_deg"].max(),
        valinit=init_roll,
    )

    # 3. Update Function
    def update(val):
        p = s_pitch.val
        r = s_roll.val

        # Update Predicted Curve
        pred_line.set_ydata(predict_wrmse(p, r, yaw_range))

        # Find Nearest Empirical Data
        # Calculate distance to all available (pitch, roll) pairs
        df_unique = df[["cam_pitch_deg", "cam_roll_deg"]].drop_duplicates()
        distances = np.sqrt(
            (df_unique["cam_pitch_deg"] - p) ** 2 + (df_unique["cam_roll_deg"] - r) ** 2
        )
        nearest_idx = distances.idxmin()

        nearest_p = df_unique.loc[nearest_idx, "cam_pitch_deg"]
        nearest_r = df_unique.loc[nearest_idx, "cam_roll_deg"]

        # Filter df for these nearest coordinates
        empirical_data = df[
            (df["cam_pitch_deg"] == nearest_p) & (df["cam_roll_deg"] == nearest_r)
        ]
        empirical_data = empirical_data.sort_values("car_yaw_deg")

        emp_dots.set_data(
            empirical_data["car_yaw_deg"], empirical_data["weighted_rmse"]
        )

        ax.set_title(f"Nearest Empirical: Pitch={nearest_p}°, Roll={nearest_r}°")
        fig.canvas.draw_idle()

    # Register the update function with the sliders
    s_pitch.on_changed(update)
    s_roll.on_changed(update)

    # Call update once to initialize the empirical dots
    update(None)

    plt.show()


def p_wrmse_vs_yaw(df):
    fig, ax = plt.subplots()

    for index, group in df.groupby(["misalignment_index"]):
        group = group.sort_values("car_yaw_deg")
        ax.plot(group["car_yaw_deg"], group["weighted_rmse"])

    ax.set_xlabel("Car Yaw [deg]")
    ax.set_ylabel("WRMSE [deg]")
    ax.grid()

    fig.savefig("wrmse_vs_yaw.png")
    plt.close()


def load():
    path = "tilt_effect_results.csv"
    df = pd.read_csv(path)

    # find mean abs wrmse

    df["cam_yaw_rad"] = np.deg2rad(df["cam_yaw_deg"])
    df["cam_pitch_rad"] = np.deg2rad(df["cam_pitch_deg"])
    df["cam_roll_rad"] = np.deg2rad(df["cam_roll_deg"])

    df["car_yaw_rad"] = np.deg2rad(df["car_yaw_deg"])
    df["car_pitch_rad"] = np.deg2rad(df["car_pitch_deg"])
    df["car_roll_rad"] = np.deg2rad(df["car_roll_deg"])

    return df


if __name__ == "__main__":
    main()
