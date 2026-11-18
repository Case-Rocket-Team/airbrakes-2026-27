import ControlBind


if __name__ == '__main__':
    mpc = ControlBind.MpcParameters()
    mpc.extension_step = 0.05
    mpc.prediction_dt = 0.1
    mpc.target_apogee_ft = 10000.0

    aero = ControlBind.AeroParameters()
    aero.base_drag_coefficient = 0.5
    aero.max_brake_delta_cd = 0.3

    actuator = ControlBind.ActuatorParameters()
    actuator.base_drag_coefficient = 0.2  # This is bound to max_slew_rate in C++

    rocket = ControlBind.RocketParameters()
    rocket.reference_area_ft2 = 1.0
    rocket.mass_slug = 1.0

    state = ControlBind.VerticalState()
    state.altitude_ft = 5000.0
    state.vertical_velocity_fps = 300.0


    result = ControlBind.slew_mpc(state, rocket, aero, mpc, .78, actuator)
    print(result.extension)


# test_bindings.py
# import your_module as m
