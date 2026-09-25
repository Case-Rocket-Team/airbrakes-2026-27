import matplotlib
matplotlib.use("TkAgg")

import matplotlib.pyplot as plt
from rocketpy import SolidMotor, Rocket, Environment, Flight

#envirment var


env = Environment(latitude=32.990254, longitude=-106.974998, elevation=1400)
import datetime
tomorrow = datetime.date.today() + datetime.timedelta(days=1)
env.set_date(
    (tomorrow.year, tomorrow.month, tomorrow.day, 12)
)  # Hour given in UTC time
env.set_atmospheric_model(type="Forecast", file="GFS")

env.info()



# Your motor
example_solid = SolidMotor(
    thrust_source="./data/Cesaroni_M1670.eng",
    dry_mass=1.815,
    dry_inertia=(0.125, 0.125, 0.002),
    nozzle_radius=33 / 1000,
    grain_number=5,
    grain_density=1815,
    grain_outer_radius=33 / 1000,
    grain_initial_inner_radius=15 / 1000,
    grain_initial_height=120 / 1000,
    grain_separation=5 / 1000,
    grains_center_of_mass_position=0.397,
    center_of_dry_mass_position=0.317,
    nozzle_position=0,
    burn_time=3.9,
    throat_radius=11 / 1000,
    coordinate_system_orientation="nozzle_to_combustion_chamber",
)

# Save original show
# _original_show = plt.show

# Non-blocking show, but force the GUI to draw
# def show_all(*args, **kwargs):
#     kwargs["block"] = False
#     _original_show(*args, **kwargs)
#
#     # Force every figure to actually render
#     for fig_num in plt.get_fignums()[1:]:
#       print(fig_num)
#       fig = plt.figure(fig_num)
#       fig.canvas.draw()
#       fig.canvas.flush_events()
#
#
# plt.show = show_all

# Generate RocketPy's figures
# example_solid.all_info()

# Give Tkinter time to finish drawing
# plt.pause(1)

# input("Press Enter to close all figures...")

# plt.close("all")



calisto = Rocket(
    radius=127 / 2000,
    mass=14.426,
    inertia=(6.321, 6.321, 0.034),
    power_off_drag="./data/powerOffDragCurve.csv",
    power_on_drag="./data/powerOnDragCurve.csv",
    center_of_mass_without_motor=0,
    coordinate_system_orientation="tail_to_nose",
)
calisto.add_motor(example_solid, position=-1.255)

rail_buttons = calisto.set_rail_buttons(
    upper_button_position=0.0818,
    lower_button_position=-0.6182,
    angular_position=45,
)
nose_cone = calisto.add_nose(
    length=0.55829, kind="von karman", position=1.278
)

fin_set = calisto.add_trapezoidal_fins(
    n=4,
    root_chord=0.120,
    tip_chord=0.060,
    span=0.110,
    position=-1.04956,
    cant_angle=0.5,
    airfoil=("./data/NACA0012-radians.txt","radians"),
)

tail = calisto.add_tail(
    top_radius=0.0635, bottom_radius=0.0435, length=0.060, position=-1.194656
)
main = calisto.add_parachute(
    name="main",
    cd_s=10.0,
    trigger=800,      # ejection altitude in meters
    sampling_rate=105,
    lag=1.5,
    noise=(0, 8.3, 0.5),
    radius=1.5,
    height=1.5,
    porosity=0.0432,
)

drogue = calisto.add_parachute(
    name="drogue",
    cd_s=1.0,
    trigger="apogee",  # ejection at apogee
    sampling_rate=105,
    lag=1.5,
    noise=(0, 8.3, 0.5),
    radius=1.5,
    height=1.5,
    porosity=0.0432,
)
calisto.draw()

test_flight = Flight(
    rocket=calisto, environment=env, rail_length=5.2, inclination=85, heading=0
    )
# init\
test_flight.prints.initial_conditions()
test_flight.prints.surface_wind_conditions()

#flight data
test_flight.prints.apogee_conditions()
test_flight.prints.impact_conditions()

#other
test_flight.prints.events_registered()
test_flight.prints.maximum_values()

#3d view
test_flight.plots.trajectory_3d()