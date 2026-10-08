# Generated from: simulation.ipynb
# Converted at: 2026-10-08T22:53:45.538Z
# Next step (optional): refactor into modules & generate tests with RunCell
# Quick start: pip install runcell

# # RocketPy Simulation
# This notebook was generated using Rocket-Serializer, a RocketPy tool to convert simulation files to RocketPy simulations
# The notebook was generated using the following parameters file: `/home/jcg/Documents/school/CWRU/26F/rocket/github/AB_2/parameters.json`
# 


# %pip install rocketpy<=2.0


from rocketpy import Environment, SolidMotor, Rocket, Flight, TrapezoidalFins, EllipticalFins, RailButtons, NoseCone, Tail, Parachute
import datetime


# ## Environment
# 


env = Environment()
env.set_location(latitude=28.61, longitude=-80.6)
env.set_elevation(0.0)


# Optionally, you can set the date and atmospheric model
# 


tomorrow = datetime.date.today() + datetime.timedelta(days=1)
env.set_date((tomorrow.year, tomorrow.month, tomorrow.day, 12))
# env.set_atmospheric_model(type='Forecast', file='GFS')

# env.all_info()


# ## Motor
# Currently, only Solid Motors are supported by Rocket-Serializer. If you want to use a Liquid/Hybrid motor, please use rocketpy directly.
# 


motor = SolidMotor(
    thrust_source='thrust_source.csv',
    dry_mass=0,
    center_of_dry_mass_position=0,
    dry_inertia=[0, 0, 0],
    grains_center_of_mass_position=0,
    grain_number=1,
    grain_density=1283.5584377471948,
    grain_outer_radius=0.027,
    grain_initial_inner_radius=0.0135,
    grain_initial_height=0.635,
    grain_separation=0,
    nozzle_radius=0.02025,
    nozzle_position=-0.3175,
    throat_radius=0.0135,
    reshape_thrust_curve=False,  # Not implemented in Rocket-Serializer
    interpolation_method='linear',
    coordinate_system_orientation='nozzle_to_combustion_chamber',
)


# motor.all_info()


# ## Rocket
# Currently, only single stage rockets are supported by Rocket-Serializer
# We will start by defining the aerodynamic surfaces, and then build the rocket.
# 


# ### Nosecones
# 


nosecone = NoseCone(
    length=0.563118,
    kind='Von Karman',
    base_radius=0.0511048,
    rocket_radius=0.0511048,
    name='0.563118',
)


# ### Fins
# As rocketpy allows for multiple fins sets, we will create a dictionary with all the fins sets and then add them to the rocket
# 


trapezoidal_fins = {}


trapezoidal_fins[0] = TrapezoidalFins(
    n=4,
    root_chord=0.1651,
    tip_chord=0.08889999999999999,
    span=0.127,
    cant_angle=0.0,
    sweep_length= 0.07619999999999999,
    sweep_angle= None,
    rocket_radius=0.0511048,
    name='Trapezoidal Fin Set',
)



# ### Transitions (Tails)
# As rocketpy allows for multiple tails, we will create a dictionary with all the tails and then add them to the rocket
# 


tails = {}


tails[0] = Tail(
    top_radius=0.0511048,
    bottom_radius=0.030607000000000002,
    length=0.1143,
    rocket_radius=0.0511048,
    name='Transition',
)


# ### Parachutes
# As rocketpy allows for multiple parachutes, we will create a dictionary with all the parachutes and then add them to the rocket
# 


parachutes = {}


parachutes[0] = Parachute(
    name='drogue',
    cd_s=0.368,
    trigger='apogee',
    sampling_rate=100, 
)


parachutes[1] = Parachute(
    name='SkyAngle Classic 52 [Cd 1.46 (9 oz) 77.8 in^3]',
    cd_s=0.912,
    trigger=228.600,
    sampling_rate=100, 
)


rocket = Rocket(
    radius=0.0511048,
    mass=7.974,
    inertia=[0.011, 0.011, 2.051],
    power_off_drag='drag_curve.csv',
    power_on_drag='drag_curve.csv',
    center_of_mass_without_motor=1.321,
    coordinate_system_orientation='nose_to_tail',
)


# ### Adding surfaces to the rocket
# Now that we have all the surfaces, we can add them to the rocket
# 


rocket.add_surfaces(surfaces=[nosecone, trapezoidal_fins[0], tails[0]], positions=[0.0, 1.78, 1.931543])

rocket.add_motor(motor, position= 1.7296869192280195)


# Adding parachutes to the rocket
# 

rocket.set_rail_buttons(
    upper_button_position=1.54,
    lower_button_position=1.90,
)
rocket.parachutes = list(parachutes.values())


# ### Rail Buttons
# 


# No rail buttons were added to the rocket.

### Rocket Info
rocket.draw()


# ## Flight
# We will now create the flight simulation. Let's go!
# 


flight = Flight(
    rocket=rocket,
    environment=env,
    rail_length=1,
    inclination=90.0,
    heading=90.0,
    terminate_on_apogee=False,
    max_time=600,
)

flight.plots.trajectory_3d()

print(f"out of rail time: {flight.out_of_rail_time}"),

# ## Compare Results
# We will now compare the results of the simulation with the parameters used to create it. Let's go!
# 


### OpenRocket vs RocketPy Parameters
time_to_apogee_ork = 21.2
time_to_apogee_rpy = flight.apogee_time
print(f"Time to apogee (OpenRocket): {time_to_apogee_ork:.3f} s")
print(f"Time to apogee (RocketPy):   {time_to_apogee_rpy:.3f} s")
apogee_difference = time_to_apogee_rpy - time_to_apogee_ork
error = abs((apogee_difference)/time_to_apogee_rpy)*100
print(f"Time to apogee difference:   {error:.3f} %")
print()

apogee_ork = 2378
apogee_rpy = flight.apogee
print(f"Apogee (OpenRocket): {apogee_ork:.3f} s")
print(f"Apogee (RocketPy):   {apogee_rpy:.3f} s")
apogee_difference = apogee_rpy - apogee_ork
error = abs((apogee_difference)/apogee_rpy)*100
print(f"Apogee difference:   {error:.3f} %")
print()

flight_time_ork = 153.226
flight_time_rpy = flight.t_final
print(f"Flight time (OpenRocket): {flight_time_ork:.3f} s")
print(f"Flight time (RocketPy):   {flight_time_rpy:.3f} s")
flight_time_difference = flight_time_rpy - flight_time_ork
error_flight_time = abs((flight_time_difference)/flight_time_rpy)*100
print(f"Flight time difference:   {error_flight_time:.3f} %")
print()

ground_hit_velocity_ork = -10.141
ground_hit_velocity_rpy = flight.impact_velocity
print(f"Ground hit velocity (OpenRocket): {ground_hit_velocity_ork:.3f} m/s")
print(f"Ground hit velocity (RocketPy):   {ground_hit_velocity_rpy:.3f} m/s")
ground_hit_velocity_difference = ground_hit_velocity_rpy - ground_hit_velocity_ork
error_ground_hit_velocity = abs((ground_hit_velocity_difference)/ground_hit_velocity_rpy)*100
print(f"Ground hit velocity difference:   {error_ground_hit_velocity:.3f} %")
print()

launch_rod_velocity_ork = 16.498
launch_rod_velocity_rpy = flight.out_of_rail_velocity
print(f"Launch rod velocity (OpenRocket): {launch_rod_velocity_ork:.3f} m/s")
print(f"Launch rod velocity (RocketPy):   {launch_rod_velocity_rpy:.3f} m/s")
launch_rod_velocity_difference = launch_rod_velocity_rpy - launch_rod_velocity_ork
error_launch_rod_velocity = abs((launch_rod_velocity_difference)/launch_rod_velocity_rpy)*100
print(f"Launch rod velocity difference:   {error_launch_rod_velocity:.3f} %")
print()

max_acceleration_ork = 131.6
max_acceleration_rpy = flight.max_acceleration
print(f"Max acceleration (OpenRocket): {max_acceleration_ork:.3f} m/s²")
print(f"Max acceleration (RocketPy):   {max_acceleration_rpy:.3f} m/s²")
max_acceleration_difference = max_acceleration_rpy - max_acceleration_ork
error_max_acceleration = abs((max_acceleration_difference)/max_acceleration_rpy)*100
print(f"Max acceleration difference:   {error_max_acceleration:.3f} %")
print()

max_altitude_ork = 2379.927
max_altitude_rpy = flight.apogee - flight.env.elevation
print(f"Max altitude (OpenRocket): {max_altitude_ork:.3f} m")
print(f"Max altitude (RocketPy):   {max_altitude_rpy:.3f} m")
max_altitude_difference = max_altitude_rpy - max_altitude_ork
error_max_altitude = abs((max_altitude_difference)/max_altitude_rpy)*100
print(f"Max altitude difference:   {error_max_altitude:.3f} %")
print()

max_mach_ork = 0.77
max_mach_rpy = flight.max_mach_number 
print(f"Max Mach (OpenRocket): {max_mach_ork:.3f}")
print(f"Max Mach (RocketPy):   {max_mach_rpy:.3f}")
max_mach_difference = max_mach_rpy - max_mach_ork
error_max_mach = abs((max_mach_difference)/max_mach_rpy)*100
print(f"Max Mach difference:   {error_max_mach:.3f} %")
print()

max_velocity_ork = 260.937
max_velocity_rpy = flight.max_speed
print(f"Max velocity (OpenRocket): {max_velocity_ork:.3f} m/s")
print(f"Max velocity (RocketPy):   {max_velocity_rpy:.3f} m/s")
max_velocity_difference = max_velocity_rpy - max_velocity_ork
error_max_velocity = abs((max_velocity_difference)/max_velocity_rpy)*100
print(f"Max velocity difference:   {error_max_velocity:.3f} %")
print()

max_thrust_ork = 1322.279
max_thrust_rpy = flight.rocket.motor.thrust.max
print(f"Max thrust (OpenRocket): {max_thrust_ork:.3f} N")
print(f"Max thrust (RocketPy):   {max_thrust_rpy:.3f} N")
max_thrust_difference = max_thrust_rpy - max_thrust_ork
error_max_thrust = abs((max_thrust_difference)/max_thrust_rpy)*100
print(f"Max thrust difference:   {error_max_thrust:.3f} %")
print()

burnout_stability_margin_ork = 2.847
burnout_stability_margin_rpy = flight.stability_margin(flight.rocket.motor.burn_out_time)
print(f"Burnout stability margin (OpenRocket): {burnout_stability_margin_ork:.3f}")
print(f"Burnout stability margin (RocketPy):   {burnout_stability_margin_rpy:.3f}")
burnout_stability_margin_difference = burnout_stability_margin_rpy - burnout_stability_margin_ork
error_burnout_stability_margin = abs((burnout_stability_margin_difference)/burnout_stability_margin_rpy)*100
print(f"Burnout stability margin difference:   {error_burnout_stability_margin:.3f} %")
print()

max_stability_margin_ork = 2.927
max_stability_margin_rpy = flight.max_stability_margin
print(f"Max stability margin (OpenRocket): {max_stability_margin_ork:.3f}")
print(f"Max stability margin (RocketPy):   {max_stability_margin_rpy:.3f}")
max_stability_margin_difference = max_stability_margin_rpy - max_stability_margin_ork
error_max_stability_margin = abs((max_stability_margin_difference)/max_stability_margin_rpy)*100
print(f"Max stability margin difference:   {error_max_stability_margin:.3f} %")
print()

min_stability_margin_ork = 0.0
min_stability_margin_rpy = flight.min_stability_margin
print(f"Min stability margin (OpenRocket): {min_stability_margin_ork:.3f}")
print(f"Min stability margin (RocketPy):   {min_stability_margin_rpy:.3f}")
min_stability_margin_difference = min_stability_margin_rpy - min_stability_margin_ork
error_min_stability_margin = abs((min_stability_margin_difference)/min_stability_margin_rpy)*100
print(f"Min stability margin difference:   {error_min_stability_margin:.3f} %")
print()