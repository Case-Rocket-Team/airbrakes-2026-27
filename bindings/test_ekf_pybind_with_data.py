"""

File ork_process.py
-------------------

Processes output csv from an Open Rocket simulation export into a binary file
that is easier to work with in C. 

"""

import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import math
import sys
import utils.ork_process as ork_process

from pyquaternion import Quaternion

import PythonBindingEKF


def to_vec3f(value):
    x, y, z = np.asarray(value, dtype=np.float32).reshape(3)
    return PythonBindingEKF.vec3f(float(x), float(y), float(z))
def to_quatf(value):
    w, x, y, z = np.asarray(value, dtype=np.float32).reshape(4)
    return PythonBindingEKF.quatf(float(w), float(x), float(y), float(z))



def process_6dof_EKF_test(full_data):
    times, attitudes, pos_ft, vel_fps, gyros_radps, accelerometer_fps2 = ork_process.process_6dof(full_data)
    # init variables run
    world_north = PythonBindingEKF.vec3f(0.0, 1.0, 0.0)
    settings = PythonBindingEKF.ekf_settings(
            0.011196358441216, 0.0001, 1.09662e-06, 1e-05, 100.0,
            # [0.0, 1.0, 0.0],
            [1.024e-05, 0.0, 0.0, 0.0, 1.024e-05, 0.0, 0.0, 0.0, 1.681e-05],
            1e-05,
            [18.0, 0.0, 0.0, 0.0, 18.0, 0.0, 0.0, 0.0, 200.0]
    )
    plot_times = []
    pos_z = []
    acu_pos_z = []

    #init
    ekf = PythonBindingEKF.ExtendedKalmanFilter(settings, PythonBindingEKF.vec3f(0.0, 0.0, -1.0), world_north, 0)
    n = min(len(times), len(accelerometer_fps2), len(gyros_radps))


    for j in range(n):
            ts = int(times[j] * 1e6)

            print("time: ", ts)
            ekf.inject_imu(to_vec3f(accelerometer_fps2[j]), to_vec3f(gyros_radps[j]), ts)
            ekf.inject_baro(pos_ft[j][2], ts)
            world_to_body = PythonBindingEKF.quatf_conj(to_quatf(attitudes[j]))
            ekf.inject_magn(PythonBindingEKF.quatf_rot_vec3f(world_to_body, world_north), ts)
            ekf.inject_gnss(to_vec3f(pos_ft[j]), ts)

            print("nominal state:", ekf.get_nominal_state().pos_ft.z)
            print("actual state:", pos_ft[j][2])


            state = ekf.get_nominal_state()
            plot_times.append(ts / 1e6)  # seconds
            pos_z.append(state.pos_ft.z)
            acu_pos_z.append(pos_ft[j][2])

    fig, ax1 = plt.subplots()

    ax1.plot(plot_times, pos_z, color="tab:blue", label="Z position (ft)")
    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("Z position (ft)", color="tab:blue")
    ax1.tick_params(axis="y", labelcolor="tab:blue")
    ax1.grid(True)

    ax2 = ax1.twinx()
    ax2.plot(plot_times, acu_pos_z, color="tab:orange", label="Z velocity (ft/s)")
    ax2.set_ylabel("Z velocity (ft/s)", color="tab:orange")
    ax2.tick_params(axis="y", labelcolor="tab:orange")

    ax1.set_title("EKF Z Position and Velocity vs. Time")
    fig.tight_layout()
    plt.show()




if __name__ == "__main__":
    full_data = pd.read_csv(
        "../ork_flight_30k.csv" if len(sys.argv) <= 1 else sys.argv[1],
        comment="#"
    )

    process_6dof_EKF_test(full_data)
