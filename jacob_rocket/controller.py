def controller_function(time, sampling_rate, state, state_history,
                        observed_variables, air_brakes, sensors, environment):
    if time > 6.0:   # set to your motor's burn time
        air_brakes.deployment_level = 0
    else:
        air_brakes.deployment_level = 0
    return (time, air_brakes.deployment_level, 0)