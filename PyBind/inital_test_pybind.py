import numpy as np
import PyBind


if __name__ == '__main__':
    #two ways to make settings
    #one is using the function; like below
    settings = PyBind.ekf_settings(
        0.011196358441216, 0.0001, 1.09662e-06, 1e-05, 100.0,
        # [0.0, 1.0, 0.0],
        [1.024e-05, 0.0, 0.0, 0.0, 1.024e-05, 0.0, 0.0, 0.0, 1.681e-05],
        1e-05,
        [18.0, 0.0, 0.0, 0.0, 18.0, 0.0, 0.0, 0.0, 200.0]
    )

    #this is another way to use the settings
    settings2= PyBind.ekf_settings()
    settings2.accel_var_f2ps4 = 0.011196358441216
    settings2.accel_bias_var_f2ps4 =  0.0001
    settings2.gyro_var_rad2ps2 =  1.09662e-06
    settings2.gyro_bias_var_rad2ps2 = 1e-05
    settings2.baro_var_ft2 = 100.0
    settings2.magn_covar_gauss2 = [1.024e-05, 0.0, 0.0, 0.0, 1.024e-05, 0.0, 0.0, 0.0, 1.681e-05]
    settings2.magn_bias_var_gauss2 = 1e-05
    settings2.gnss_covar_ft2 = [18.0, 0.0, 0.0, 0.0, 18.0, 0.0, 0.0, 0.0, 200.0]

    #init
    ekf = PyBind.EKF()
    print(ekf.get_nominal_state().attitude.w)
    ekf.init(settings, PyBind.vec3f(0.0, 0.0, -1.0), PyBind.vec3f(0.0, 1.0, 0.0), 0)
    #just to check that init works
    print(ekf.get_nominal_state().attitude.w)


