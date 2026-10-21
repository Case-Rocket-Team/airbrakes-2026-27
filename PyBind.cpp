//Copyright(c) 2026 Jmaster101.All Rights Reserved;

#include <pybind11/pybind11.h>
#include "PyBind.h"
#include "kalman_filter/extended_kalman_filter.h"
#include <pybind11/stl.h>
#include <pybind11/numpy.h>

namespace py = pybind11;

Test::Test(me2 *c) {
    data = *c;
}

Test::Test(float dat, int version) {
    data = {dat, version,};
}

me2 Test::getVar() const {
    return data;
}

std::tuple<float, int> Test::getTuple() const {
    return {data.version, data.version};
}

static ekf_settings setSettings(f32 accel_var_f2ps4, f32 accel_bias_var_f2ps4, f32 gyro_var_rad2ps2, f32 gyro_bias_var_rad2ps2,
                               f32 baro_var_ft2, std::array<f32, 3> &world_magn_north_guass, std::array<f32, 9> &magn_covar_gauss2, f32 magn_bias_var_gauss2,
                               std::array<f32, 9> &gnss_covar_ft2) {

    //setup
    printf("hello from C++\n");
    ekf_settings settings{};

    //set the ones that don't cause issues
    settings.accel_var_f2ps4 = accel_var_f2ps4;
    settings.accel_bias_var_f2ps4 = accel_bias_var_f2ps4;
    settings.gyro_var_rad2ps2 = gyro_var_rad2ps2;
    settings.gyro_bias_var_rad2ps2 = gyro_bias_var_rad2ps2;
    settings.baro_var_ft2 = baro_var_ft2;
    settings.magn_bias_var_gauss2 = magn_bias_var_gauss2;

    //set the array to work
    for (int i = 0; i < 9; i++) {
        settings.magn_covar_gauss2[i] = magn_covar_gauss2[i];
        settings.gnss_covar_ft2[i] = gnss_covar_ft2[i];
    }
    return settings;
}
vec3f loadvec3f(float x, float y, float z) {
    return  {x, y, z};
}


PYBIND11_MODULE(PyBind, m) {
    m.def("ekf_init", &ekf_init);
    py::class_<me2>(m, "Me2")
       .def(py::init<>())
       .def_readwrite("dat", &me2::dat)
       .def_readwrite("version", &me2::version);

    py::class_<Test>(m, "Test")
        .def(py::init<float, int>())
        .def("get",&Test::getVar)
        .def("getTuple",&Test::getTuple);
        // .def("setSettings",&Test::setSettings);

    py::class_<ekf_nominal_state>(m, "ekf_nominal_state")
      .def(py::init<>())
      .def_readwrite("attitude", &ekf_nominal_state::attitude)
      .def_readwrite("vel_fps", &ekf_nominal_state::vel_fps)
      .def_readwrite("pos_ft", &ekf_nominal_state::pos_ft)
      .def_readwrite("gyro_bias_radps", &ekf_nominal_state::gyro_bias_radps)
      .def_readwrite("accel_bias_fps2", &ekf_nominal_state::accel_bias_fps2)
      .def_readwrite("magn_bias_gauss", &ekf_nominal_state::magn_bias_gauss);

    py::class_<ekf_settings>(m, "ekf_settings")
        .def(py::init<>())
        .def(py::init(&setSettings), py::arg("accel_var_f2ps4"), py::arg("accel_bias_var_f2ps4"),
             py::arg("gyro_var_rad2ps2"), py::arg("gyro_bias_var_rad2ps2"),
             py::arg("baro_var_ft2"), py::arg("world_magn_north_guass"),
             py::arg("magn_covar_gauss2"), py::arg("magn_bias_var_gauss2"),
             py::arg("gnss_covar_ft2"))
        .def_readwrite("accel_var_f2ps4", &ekf_settings::accel_var_f2ps4)
        .def_readwrite("accel_bias_var_f2ps4", &ekf_settings::accel_bias_var_f2ps4)
        .def_readwrite("gyro_var_rad2ps2", &ekf_settings::gyro_var_rad2ps2)
        .def_readwrite("gyro_bias_var_rad2ps2", &ekf_settings::gyro_bias_var_rad2ps2)
        .def_readwrite("baro_var_ft2", &ekf_settings::baro_var_ft2)
        .def_readwrite("magn_bias_var_gauss2", &ekf_settings::magn_bias_var_gauss2)
        .def_property(
            "gnss_covar_ft2",
            [](const ekf_settings &s) {
                  return py::array_t<float>({9}, {sizeof(float)}, s.gnss_covar_ft2);
              },
              [](ekf_settings &s, py::array_t<float> arr) {
                  auto buf = arr.request();
                  if (buf.size != 9) throw std::runtime_error("gnss_covar_ft2 must have 9 elements");
                  std::memcpy(s.gnss_covar_ft2, buf.ptr, 9 * sizeof(float));
              }
              )
        .def_property(
              "magn_covar_gauss2",
              [](const ekf_settings &s) {
                  return py::array_t<float>({9}, {sizeof(float)}, s.magn_covar_gauss2);
              },
              [](ekf_settings &s, py::array_t<float> arr) {
                  auto buf = arr.request();
                  if (buf.size != 9) throw std::runtime_error("magn_covar_gauss2 must have 9 elements");
                  std::memcpy(s.magn_covar_gauss2, buf.ptr, 9 * sizeof(float));
              }
              );

    py::class_<ekf_control_input>(m, "ekf_control_input")
      .def(py::init<>())
      .def_readwrite("accel_fps2", &ekf_control_input::accel_fps2)
      .def_readwrite("gyro_radps", &ekf_control_input::gyro_radps)
      .def_property(
           "v",
           [](const ekf_control_input &s) {
               return py::array_t<float>({6}, {sizeof(float)}, s.v);
           },
           [](ekf_control_input &s, py::array_t<float> arr) {
               auto buf = arr.request();
               if (buf.size != 6) throw std::runtime_error("magn_covar_gauss2 must have 6 elements");
               std::memcpy(s.v, buf.ptr, 6 * sizeof(float));
           }
           );

    py::class_<extended_kalman_filter>(m, "extended_kalman_filter")
        .def(py::init<>())
        .def_readwrite("nominal_state", &extended_kalman_filter::nominal_state)
        .def_readwrite("settings", &extended_kalman_filter::settings)
        .def_readwrite("state_time_us", &extended_kalman_filter::state_time_us)
        .def_readwrite("control_input", &extended_kalman_filter::control_input)
        .def_property(
            "state_covar",  [](const extended_kalman_filter &s) {
                   return py::array_t<float>({EKF_STATE_DIM * EKF_STATE_DIM}, {sizeof(float)}, s.state_covar);
               },[](extended_kalman_filter &s, py::array_t<float> arr) {
                   auto buf = arr.request();
                   if (buf.size != EKF_STATE_DIM * EKF_STATE_DIM) throw std::runtime_error("magn_covar_gauss2 must have 6 elements");
                   std::memcpy(s.state_covar, buf.ptr, EKF_STATE_DIM * EKF_STATE_DIM* sizeof(float));
               }
               );

    py::class_<vec3f>(m, "vec3f")
        .def(py::init(&loadvec3f))
        .def_readwrite("x", &vec3f::x)
        .def_readwrite("y", &vec3f::y)
        .def_readwrite("z", &vec3f::z);
}
