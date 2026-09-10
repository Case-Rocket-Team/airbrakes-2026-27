import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

def main():
    df = pd.read_csv("kf_results.csv")

    fig, axs = plt.subplots(3, 2)

    t = df["time_s"]

    axs[0][0].plot(t, df["altitude_ft"], label="real")
    axs[0][0].plot(t, df["kf_altitude_ft"], label="kalman")
    axs[0][0].plot(t, df["m_altitude_ft"], '.', markersize=1, label="measured")
    axs[0][0].legend()
    axs[0][0].set_ylabel("altitude (ft)")

    axs[0][1].plot(t, df["kf_altitude_ft"] - df["altitude_ft"])
    axs[0][1].axhline(y=0, ls="--")
    axs[0][1].set_ylabel("altitude error (ft)")

    axs[1][0].plot(t, df["vertical_vel_fps"], label="real")
    axs[1][0].plot(t, df["kf_vertical_vel_fps"], label="kalman")
    axs[1][0].legend()
    axs[1][0].set_ylabel("vertical velocity (ft/s)")

    axs[1][1].plot(t, df["kf_vertical_vel_fps"] - df["vertical_vel_fps"])
    axs[1][1].axhline(y=0, ls="--")
    axs[1][1].set_ylabel("velocity error (ft/s)")

    axs[2][0].plot(t, df["accel_bias_fps2"], label="real")
    axs[2][0].plot(t, df["kf_accel_bias_fps2"], label="kalman")
    axs[2][0].legend()
    axs[2][0].set_ylabel("accel bias (ft/s^2)")

    axs[2][1].plot(t, df["kf_accel_bias_fps2"] - df["accel_bias_fps2"])
    axs[2][1].axhline(y=0, ls="--")
    axs[2][1].set_ylabel("accel bias error (ft/s^2)")

    plt.show()

if __name__ == "__main__":
    main()
