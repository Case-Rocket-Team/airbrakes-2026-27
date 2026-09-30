"""

File ork_process.py
-------------------

Processes output csv from an Open Rocket simulation export into a binary file
that is easier to work with in C. 

TODO: process ork format into 3d vectors for velocity and acceleration

"""

import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import math
import sys

from pyquaternion import Quaternion

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
    out = np.zeros((nrows, 4))

    attitude = Quaternion(axis=(0, 1, 0), degrees=6)

    for i in range(nrows):
        out[i,:] = attitude.elements

        attitude_diff = 0.5 * dts[i] * attitude * \
            Quaternion(real=0, imaginary=gyros_radps[i,:])
        attitude = (attitude + attitude_diff).unit

    return out.astype(np.float32)

def main():
    full_data = pd.read_csv(
        "ork_flight_30k.csv" if len(sys.argv) <= 1 else sys.argv[1],
        comment="#"
    )

    def col(s):
        return full_data[s].to_numpy().astype(np.float32)

    times = col("Time (s)")
    dts = col("Simulation time step (s)")

    pos_ft = np.array([
        col("Position East of launch (ft)"),
        col("Position North of launch (ft)"),
        col("Altitude (ft)") ,
    ]).T

    gyros_radps = 2 * math.pi * np.array([
        col("Pitch rate (r/s)"),
        col("Yaw rate (r/s)"),
        col("Roll rate (r/s)")
    ]).T

    nrows = int(np.where(np.isnan(gyros_radps))[0][0])

    attitudes = integrate_attitudes(nrows, dts, gyros_radps)

    with open("ork_processed.bin", "wb") as f:
        f.write(nrows.to_bytes(4, byteorder="little", signed=False))

        times[:nrows].tofile(f)
        pos_ft[:nrows].tofile(f)
        attitudes[:nrows].tofile(f)

if __name__ == "__main__":
    main()

