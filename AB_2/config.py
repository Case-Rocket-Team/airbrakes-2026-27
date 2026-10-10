from pathlib import Path

 
# --- Environment -----------------------------------------------------------
LATITUDE = 28.61
LONGITUDE = -80.6
ELEVATION = 0.0

# --- Launch / flight -------------------------------------------------------
RAIL_LENGTH = 1.0
RAIL_INCLINATION = 90.0
RAIL_HEADING = 90.0
MAX_SIM_TIME = 600
TERMINATE_ON_APOGEE = False


# --- Airbrakes -------------------------------------------------------------
# Deployed flap area in m^2. Use None to default to the rocket's cross-section.
AIRBRAKES_REFERENCE_AREA = 0.002 # random number oopsies
AIRBRAKES_SAMPLING_RATE = 20
