import numpy as np

import PyBind


if __name__ == '__main__':
    obj = PyBind.Test(7, 2)
    me = PyBind.ekf_settings(
        0.011196358441216, 0.0001, 1.09662e-06, 1e-05, 100.0,
        [0.0, 1.0, 0.0],
        [1.024e-05, 0.0, 0.0, 0.0, 1.024e-05, 0.0, 0.0, 0.0, 1.681e-05],
        1e-05,
        [18.0, 0.0, 0.0, 0.0, 18.0, 0.0, 0.0, 0.0, 200.0]
    )
    set= PyBind.ekf_settings()
    set.accel_var_f2ps4 = 0.011196358441216
    set.accel_bias_var_f2ps4 =  0.0001
    set.gyro_var_rad2ps2 =  1.09662e-06
    set.gyro_bias_var_rad2ps2 = 1e-05
    set.baro_var_ft2 = 100.0
    # set.world_magn_north_guass = test.vec3f(0.0, 1.0, 0.0)
    set.magn_covar_gauss2 = [1.024e-05, 0.0, 0.0, 0.0, 1.024e-05, 0.0, 0.0, 0.0, 1.681e-05]
    set.magn_bias_var_gauss2 = 1e-05
    set.gnss_covar_ft2 = [18.0, 0.0, 0.0, 0.0, 18.0, 0.0, 0.0, 0.0, 200.0]

    ekf = PyBind.extended_kalman_filter()
    # test.ekf_init(ekf, me, 2)
    PyBind.ekf_init(ekf, me, PyBind.vec3f(0.0, 0.0, -1.0 ), PyBind.vec3f(0.0, 1.0, 0.0), 0);
    print(me.magn_covar_gauss2);



