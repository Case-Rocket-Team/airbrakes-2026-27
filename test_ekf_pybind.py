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

from pyquaternion import Quaternion

import PyBind


def to_vec3f(value):
    x, y, z = np.asarray(value, dtype=np.float32).reshape(3)
    return PyBind.vec3f(float(x), float(y), float(z))
def to_quatf(value):
    w, x, y, z = np.asarray(value, dtype=np.float32).reshape(4)
    return PyBind.quatf(float(w), float(x), float(y), float(z))


"""

List of columns
---------------

Time (s)
Altitude (ft)
Altitude above sea level (ft)
Vertical velocity (ft/s)
Total velocity (ft/s)
Vertical acceleration (ft/s²)
Total acceleration (ft/s²)
Position East of launch (ft)
Position North of launch (ft)
Lateral distance (ft)
Lateral direction (°)
Lateral velocity (ft/s)
Lateral acceleration (ft/s²)
Latitude (° N)
Longitude (° E)
Angle of attack (°)
Roll rate (r/s)
Pitch rate (r/s)
Yaw rate (r/s)
Vertical orientation (zenith) (°)
Lateral orientation (azimuth) (°)
Mass (oz)
Motor mass (oz)
Longitudinal moment of inertia (lb·ft²)
Rotational moment of inertia (lb·ft²)
Gravitational acceleration (ft/s²)
CP location (in)
CG location (in)
Stability margin calibers (​)
Thrust (N)
Thrust-to-weight ratio (​)
Drag force (N)
Drag coefficient (​)
Friction drag coefficient (​)
Pressure drag coefficient (​)
Base drag coefficient (​)
Axial drag coefficient (​)
Normal force coefficient (​)
Pitch moment coefficient (​)
Yaw moment coefficient (​)
Side force coefficient (​)
Roll moment coefficient (​)
Roll forcing coefficient (​)
Roll damping coefficient (​)
Pitch damping coefficient (​)
Wind velocity (ft/s)
Wind direction (°)
Air temperature (°F)
Air pressure (mbar)
Air density (oz/in³)
Speed of sound (ft/s)
Mach number (​)
Reynolds number (​)
Reference length (in)
Reference area (in²)
Simulation time step (s)
Computation time (s)
Coriolis acceleration (ft/s²)
Damping moment coefficient (​)
CMC ()
NatFreq ()
CNalpha ()
Damping Ratio ()
"""


def integrate_attitudes(nrows, dts, gyros_radps):
    out = np.zeros((nrows, 4), dtype=np.float32)

    attitude = Quaternion(axis=(0, 1, 0), degrees=6)

    for i in range(nrows):
        out[i,:] = attitude.elements

        attitude_diff = attitude * Quaternion(
            real=0, imaginary=0.5 * dts[i] * gyros_radps[i,:]
        )
        attitude = (attitude + attitude_diff).unit

    return out.astype(np.float32)


def process_6dof(full_data):
    def col(s):
        return full_data[s].to_numpy().astype(np.float32)

    times = col("Time (s)")
    dts = col("Simulation time step (s)")

    pos_ft = np.array([
        col("Position East of launch (ft)"),
        col("Position North of launch (ft)"),
        col("Altitude (ft)"),
    ]).T

    gyros_radps = 2 * math.pi * np.array([
        col("Pitch rate (r/s)"),
        col("Yaw rate (r/s)"),
        col("Roll rate (r/s)")
    ]).T

    nrows = int(np.where(np.isnan(gyros_radps))[0][0])

    vertical_vel_fps = col("Vertical velocity (ft/s)")
    lateral_vel_fps = col("Lateral velocity (ft/s)")
    lateral_dir_deg = col("Lateral direction (°)") 

    total_vel_fps = col("Total velocity (ft/s)")

    vel_fps = np.zeros((nrows + 1, 3), dtype=np.float32)
    for i in range(nrows + 1):
        vel_fps[i,:] = np.array([
            lateral_vel_fps[i] * math.sin(lateral_dir_deg[i] * math.pi / 180),
            lateral_vel_fps[i] * math.cos(lateral_dir_deg[i] * math.pi / 180),
            vertical_vel_fps[i]
        ])

    # Note(Ian) Using finite difference here to get world accel because ORK 
    # does not include the acceleration vector components directly. I compared
    # the finite difference total and vertical accel with ORK data, and it is 
    # close enough that I do not think it would cuase an issue for this test
    world_accel_fps2 = np.zeros((nrows, 3), dtype=np.float32)
    for i in range(nrows):
        dt = times[i + 1] - times[i]
        world_accel_fps2[i] = ((vel_fps[i+1,:] - vel_fps[i,:]) / dt) 

    attitudes = integrate_attitudes(nrows, dts, gyros_radps)

    g = np.array([0, 0, 32.174])
    accelerometer_fps2 = np.zeros((nrows, 3), dtype=np.float32)
    for i in range(nrows):
        quat = Quaternion(attitudes[i,:]).conjugate
        accelerometer_fps2[i,:] = quat.rotate(world_accel_fps2[i,:] + g)



    # init for run
    world_north = PyBind.vec3f(0.0, 1.0, 0.0)
    settings = PyBind.ekf_settings(
            0.011196358441216, 0.0001, 1.09662e-06, 1e-05, 100.0,
            # [0.0, 1.0, 0.0],
            [1.024e-05, 0.0, 0.0, 0.0, 1.024e-05, 0.0, 0.0, 0.0, 1.681e-05],
            1e-05,
            [18.0, 0.0, 0.0, 0.0, 18.0, 0.0, 0.0, 0.0, 200.0]
    )
    plot_times = []
    pos_z = []
    acu_pos_z = []

    ekf = PyBind.EKF()
    ekf.init(settings, PyBind.vec3f(0.0, 0.0, -1.0), world_north, 0)
    n = min(len(times), len(accelerometer_fps2), len(gyros_radps))


    for j in range(n):
            ts = int(times[j] * 1e6)
            # print("times:", len(times))
            # print("accelerometer:", len(accelerometer_fps2))
            # print("gyros:", len(gyros_radps))
            print("time: ", ts)
            ekf.inject_imu(to_vec3f(accelerometer_fps2[j]), to_vec3f(gyros_radps[j]), ts)
            ekf.inject_baro(pos_ft[j][2], ts)
            world_to_body = PyBind.quatf_conj(to_quatf(attitudes[j]))
            ekf.inject_magn(PyBind.quatf_rot_vec3f(world_to_body, world_north), ts)
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


# with open("ork_processed_6dof.bin", "wb") as f:
    #     f.write(nrows.to_bytes(4, byteorder="little", signed=False))
    #
    #     def insert(arr):
    #         arr[:nrows].astype(np.float32).tofile(f)
    #
    #
    #     insert(times)
    #     insert(attitudes)
    #     insert(pos_ft)
    #     insert(vel_fps)
    #     insert(gyros_radps)
    #     insert(accelerometer_fps2)


def process_1d(full_data):
    def col(s):
        return full_data[s].to_numpy().astype(np.float32)

    times = col("Time (s)")
    altitudes_ft = col("Altitude (ft)")
    vertical_vel_fps = col("Vertical velocity (ft/s)")
    vertical_accel_fps2 = col("Vertical acceleration (ft/s²)")

    with open("ork_processed_1d.bin", "wb") as f:
        f.write(len(times).to_bytes(4, byteorder="little", signed=False))

        times.tofile(f)
        altitudes_ft.tofile(f)
        vertical_vel_fps.tofile(f)
        vertical_accel_fps2.tofile(f)


if __name__ == "__main__":
    full_data = pd.read_csv(
        "../ork_flight_30k.csv" if len(sys.argv) <= 1 else sys.argv[1],
        comment="#"
    )

    process_6dof(full_data)
