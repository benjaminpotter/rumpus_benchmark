import matplotlib.pyplot as plt
import pandas as pd


def main():

    path = "tilt_effect_results.csv"
    df = pd.read_csv(path)

    fig, ax = plt.subplots()

    ax.plot(df["car_yaw_deg"], df["weighted_rmse"])

    plt.show()


if __name__ == "__main__":
    main()
