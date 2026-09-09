import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import sys

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

def main():
    full_data = pd.read_csv(
        "ork_flight_30k.csv" if len(sys.argv) <= 1 else sys.argv[1],
        comment="#"
    )

    def col(s):
        return full_data[s].to_numpy().astype(np.float32)

    times = col("Time (s)")
    altitudes_ft = col("Altitude (ft)")
    vertical_vel_fps = col("Vertical velocity (ft/s)")
    vertical_accel_fps2 = col("Vertical acceleration (ft/s²)")

    with open("ork_processed.bin", "wb") as f:
        f.write(len(times).to_bytes(4, byteorder="little", signed=False))

        times.tofile(f)
        altitudes_ft.tofile(f)
        vertical_vel_fps.tofile(f)
        vertical_accel_fps2.tofile(f)

if __name__ == "__main__":
    main()

