# Generated from: simulation.ipynb
# Converted at: 2026-10-08T22:53:45.538Z
# Next step (optional): refactor into modules & generate tests with RunCell
# Quick start: pip install runcell

# # RocketPy Simulation
# This notebook was generated using Rocket-Serializer, a RocketPy tool to convert simulation files to RocketPy simulations
# The notebook was generated using the following parameters file: `/home/jcg/Documents/school/CWRU/26F/rocket/github/AB_2/parameters.json`
# 


# %pip install rocketpy<=2.0


from rocketpy import Environment, FreeFormFins, SolidMotor, Rocket, Flight, TrapezoidalFins, EllipticalFins, RailButtons, NoseCone, Tail, Parachute
from controller import controller_function;
import datetime



# ## Environment
# 


env = Environment(latitude=28.61, longitude=-80.6, elevation=0.0)

tomorrow = datetime.date.today() + datetime.timedelta(days=1)
env.set_date((tomorrow.year, tomorrow.month, tomorrow.day, 12))
#env.set_atmospheric_model(type='Forecast', file='GFS')
# env.all_info()


# ## Motor
# Currently, only Solid Motors are supported by Rocket-Serializer. If you want to use a Liquid/Hybrid motor, please use rocketpy directly.
# 


motor = SolidMotor(
    thrust_source='./data/thrust_source.csv',
    dry_mass=0,
    center_of_dry_mass_position=0,
    dry_inertia=[0, 0, 0],
    grains_center_of_mass_position=0,
    grain_number=1,
    grain_density=1608.1493427884948,
    grain_outer_radius=0.049,
    grain_initial_inner_radius=0.0245,
    grain_initial_height=1.239,
    grain_separation=0,
    nozzle_radius=0.036750000000000005,
    nozzle_position=-0.6195,
    throat_radius=0.0245,
    reshape_thrust_curve=False,  # Not implemented in Rocket-Serializer
    interpolation_method='linear',
    coordinate_system_orientation='nozzle_to_combustion_chamber',
)


# ## Rocket
# Currently, only single stage rockets are supported by Rocket-Serializer
# We will start by defining the aerodynamic surfaces, and then build the rocket.
# 


# ### Nosecones
# 


nosecone = NoseCone(
    length=0.7630159999999999,
    kind='Von Karman',
    base_radius=0.0763016,
    rocket_radius=0.0763016,
    name='0.7630159999999999',
)


# ### Fins
# As rocketpy allows for multiple fins sets, we will create a dictionary with all the fins sets and then add them to the rocket
# 

# RocketPy measures 'z' along the rocket length and 'y' outward along the span
fin_geometry = [
    (0.0, 0.0),          # Root chord leading edge
    (0.2413, 0.1778),    # Tip chord leading edge
    (0.2987, 0.1778),    # Tip chord trailing edge
    (0.2987, 0.0)        # Root chord trailing edge
]

# 2. Instantiate the FreeFormFins object
custom_fins = FreeFormFins(
    n=4,                             
    shape_points=fin_geometry,    
    rocket_radius=0.0763016,        
    cant_angle=0,                   
    airfoil=None,                    
    name="Freeform Fin Set"          
)


# ### Transitions (Tails)
# As rocketpy allows for multiple tails, we will create a dictionary with all the tails and then add them to the rocket
# 


tails = {}




tails[0] = Tail(
    top_radius=0.0763016,
    bottom_radius=0.05206999999999999,
    length=0.30479999999999996,
    rocket_radius=0.0763016,
    name='Boat Tail - 0',
)

# ### Parachutes
# As rocketpy allows for multiple parachutes, we will create a dictionary with all the parachutes and then add them to the rocket
# 


parachutes = {}

parachutes[0] = Parachute(
    name='Drogue Parachute',
    cd_s=0.475,
    trigger='apogee',
    sampling_rate=100, 
)

parachutes[1] = Parachute(
    name='Main Parachute',
    cd_s=5.385,
    trigger=457.200,
    sampling_rate=100, 
)



rocket = Rocket(
    radius=0.0763016,
    mass=19.135,
    inertia=[0.052, 0.052, 9.017],
    power_off_drag='./data/drag_curve.csv',
    power_on_drag='./data/drag_curve.csv',
    center_of_mass_without_motor=1.363,
    coordinate_system_orientation='nose_to_tail',
)


air_brakes = rocket.add_air_brakes(
    drag_coefficient_curve="./data/example_airbrakes_cd.csv",
    controller_function=controller_function,
    sampling_rate=20,
    reference_area=None,
    clamp=True,
    initial_observed_variables=[0, 0, 0],
    override_rocket_drag=False,
    name="Air Brakes",
)

#air_brakes.all_info()


# ### Adding surfaces to the rocket
# Now that we have all the surfaces, we can add them to the rocket
# 


#rocket.add_surfaces(surfaces=[nosecone, trapezoidal_fins[0], tails[0]], positions=[0.0, 1.78, 1.931543])
rocket.add_surfaces(surfaces=[nosecone, tails[0], custom_fins], positions=[0.0, 2.314956, 1.9654])

rocket.add_motor(motor, position= 1.9969287615330025)



rocket.parachutes = list(parachutes.values())
# Adding parachutes to the rocket
# 

rocket.set_rail_buttons(
    upper_button_position=1.54,
    lower_button_position=1.90,
)



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
    rail_length=1.0,
    inclination=90.0,
    heading=90.0,
    terminate_on_apogee=False,
    max_time=600,
)

flight.plots.trajectory_3d()

flight.plots.all()
# ## Compare Results
# We will now compare the results of the simulation with the parameters used to create it. Let's go!
# 


### OpenRocket vs RocketPy Parameters
# Values updated from original rocket.ork stored results block
time_to_apogee_ork = 33.745
time_to_apogee_rpy = flight.apogee_time
print(f"Time to apogee (OpenRocket): {time_to_apogee_ork:.3f} s")
print(f"Time to apogee (RocketPy):   {time_to_apogee_rpy:.3f} s")
apogee_time_difference = time_to_apogee_rpy - time_to_apogee_ork
error_apogee_time = abs((apogee_time_difference)/time_to_apogee_rpy)*100
print(f"Time to apogee difference:   {error_apogee_time:.3f} %")
print()

apogee_ork = 7269.969
apogee_rpy = flight.apogee
print(f"Apogee (OpenRocket): {apogee_ork:.3f} m")
print(f"Apogee (RocketPy):   {apogee_rpy:.3f} m")
apogee_difference = apogee_rpy - apogee_ork
error_apogee = abs((apogee_difference)/apogee_rpy)*100
print(f"Apogee difference:   {error_apogee:.3f} %")
print()

flight_time_ork = 319.372
flight_time_rpy = flight.t_final
print(f"Flight time (OpenRocket): {flight_time_ork:.3f} s")
print(f"Flight time (RocketPy):   {flight_time_rpy:.3f} s")
flight_time_difference = flight_time_rpy - flight_time_ork
error_flight_time = abs((flight_time_difference)/flight_time_rpy)*100
print(f"Flight time difference:   {error_flight_time:.3f} %")
print()

ground_hit_velocity_ork = -7.479
ground_hit_velocity_rpy = flight.impact_velocity
print(f"Ground hit velocity (OpenRocket): {ground_hit_velocity_ork:.3f} m/s")
print(f"Ground hit velocity (RocketPy):   {ground_hit_velocity_rpy:.3f} m/s")
ground_hit_velocity_difference = ground_hit_velocity_rpy - ground_hit_velocity_ork
error_ground_hit_velocity = abs((ground_hit_velocity_difference)/ground_hit_velocity_rpy)*100
print(f"Ground hit velocity difference:   {error_ground_hit_velocity:.3f} %")
print()

print(flight.max_mach_number)
launch_rod_velocity_ork = 16.870
launch_rod_velocity_rpy = flight.out_of_rail_velocity
print(f"Launch rod velocity (OpenRocket): {launch_rod_velocity_ork:.3f} m/s")
print(f"Launch rod velocity (RocketPy):   {launch_rod_velocity_rpy:.3f} m/s")
launch_rod_velocity_difference = launch_rod_velocity_rpy - launch_rod_velocity_ork
error_launch_rod_velocity = abs((launch_rod_velocity_difference)/launch_rod_velocity_rpy)*100
print(f"Launch rod velocity difference:   {error_launch_rod_velocity:.3f} %")
print()

max_acceleration_ork = 154.545
max_acceleration_rpy = flight.max_acceleration
print(f"Max acceleration (OpenRocket): {max_acceleration_ork:.3f} m/s²")
print(f"Max acceleration (RocketPy):   {max_acceleration_rpy:.3f} m/s²")
max_acceleration_difference = max_acceleration_rpy - max_acceleration_ork
error_max_acceleration = abs((max_acceleration_difference)/max_acceleration_rpy)*100
print(f"Max acceleration difference:   {error_max_acceleration:.3f} %")
print()