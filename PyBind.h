//
// Created by jcg on 10/13/26.
//

#ifndef BINDINGS_TEST_H
#define BINDINGS_TEST_H
#include <tuple>

#include "kalman_filter/extended_kalman_filter.h"

struct me2 {
    float dat;
    int version;
};

// class Test {
//     public:
//         int change{};
//         Test(int c);
//         int getVar() const;
// };

class Test {
public:
    me2 data{};
    Test(me2* c);

    Test(float dat, int version);

    me2 getVar() const;
    std::tuple<float, int> getTuple() const;

    // static ekf_settings setSettings(f32 accel_var_f2ps4, f32 accel_bias_var_f2ps4, f32 gyro_var_rad2ps2,
                             // f32 gyro_bias_var_rad2ps2,
                             // f32 baro_var_ft2, std::array<f32, 3> &world_magn_north_guass,
                             // std::array<f32, 9> &magn_covar_gauss2, f32 magn_bias_var_gauss2,
                             // std::array<f32, 9> &gnss_covar_ft2);


#endif //BINDINGS_TEST_H
};