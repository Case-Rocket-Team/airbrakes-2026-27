import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

def main():
    df = pd.read_csv("ekf_results.csv")

    fig, axs = plt.subplots(3, 2)

    t = df["time_us"] * 1e-6

    axs[0][0].plot(t, df["att_similarity"], label="Attitude Similarity")
    axs[0][0].legend()
    axs[0][0].set_ylabel("Attitude Similarity (degrees)")
    axs[0][0].set_xlabel("Time (s)")

    axs[1][0].plot(t, df["pos_err_x"], label="Position Error (x)")
    axs[1][0].plot(t, df["pos_err_y"], label="Position Error (y)")
    axs[1][0].plot(t, df["pos_err_z"], label="Position Error (z)")
    axs[1][0].axhline(y=0, ls="--")
    axs[1][0].legend()
    axs[1][0].set_ylabel("Error (ft)")
    axs[1][0].set_xlabel("Time (s)")

    axs[0][1].plot(t, df["vel_err_x"], label="Velocity Error (x)")
    axs[0][1].plot(t, df["vel_err_y"], label="Velocity Error (y)")
    axs[0][1].plot(t, df["vel_err_z"], label="Velocity Error (z)")
    axs[0][1].axhline(y=0, ls="--")
    axs[0][1].legend()
    axs[0][1].set_ylabel("Error (ft/s)")
    axs[0][1].set_xlabel("Time (s)")

    axs[1][1].plot(t, df["accel_bias_err_x"], label="Accel Bias Error (x)")
    axs[1][1].plot(t, df["accel_bias_err_y"], label="Accel Bias Error (y)")
    axs[1][1].plot(t, df["accel_bias_err_z"], label="Accel Bias Error (z)")
    axs[1][1].axhline(y=0, ls="--")
    axs[1][1].legend()
    axs[1][1].set_ylabel("Accelerometer Bias Error (ft/s^2)")
    axs[1][1].set_xlabel("Time (s)")

    axs[2][0].plot(t, df["gyro_bias_err_x"], label="Gyro Bias Error (x)")
    axs[2][0].plot(t, df["gyro_bias_err_y"], label="Gyro Bias Error (y)")
    axs[2][0].plot(t, df["gyro_bias_err_z"], label="Gyro Bias Error (z)")
    axs[2][0].axhline(y=0, ls="--")
    axs[2][0].legend()
    axs[2][0].set_ylabel("Gyroscope Bias Error (rad/s)")
    axs[2][0].set_xlabel("Time (s)")

    axs[2][1].plot(t, df["magn_bias_err_x"], label="Magnetometer Bias Error (x)")
    axs[2][1].plot(t, df["magn_bias_err_y"], label="Magnetometer Bias Error (y)")
    axs[2][1].plot(t, df["magn_bias_err_z"], label="Magnetometer Bias Error (z)")
    axs[2][1].axhline(y=0, ls="--")
    axs[2][1].legend()
    axs[2][1].set_ylabel("Magnetometer Bias Error (guass)")
    axs[2][1].set_xlabel("Time (s)")

    plt.show()

if __name__ == "__main__":
    main()
