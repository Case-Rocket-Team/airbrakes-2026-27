
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include "control/brake_command.hpp"
#include "control/actuator/actuator_model.hpp"
#include "control/controller/ideal_mpc.hpp"
#include "control/controller/slew_mpc.hpp"

namespace py = pybind11;

#include "ControlBind.h"


control::MpcParameters mcpParametersLoad(f32 extension_step, f32 prediction_dt, f32 target_apogee_ft) {
    control::MpcParameters parameters{};
    parameters.extension_step = extension_step;
    parameters.prediction_dt = prediction_dt;
    parameters.target_apogee_ft = target_apogee_ft;
    return parameters;
}

control::aero::AeroParameters aeroParametersLoad(f32 base_drag_coefficient, f32 max_brake_delta_cd) {
    control::aero::AeroParameters parameters{};
    parameters.base_drag_coefficient = base_drag_coefficient;
    parameters.max_brake_delta_cd = max_brake_delta_cd;
    return parameters;
}

control::actuator::ActuatorParameters actuatorParametersLoad(f32 max_slew_rate) {
    control::actuator::ActuatorParameters parameters{};
    parameters.max_slew_rate = max_slew_rate;
    return parameters;
}
control::physics::RocketParameters rocketParametersLoad(f32 max_slew_rate, f32 reference_area_ft2) {
    control::physics::RocketParameters parameters{};
    parameters.mass_slug = max_slew_rate;
    parameters.reference_area_ft2 = reference_area_ft2;
    return parameters;
}
control::physics::VerticalState vecticalParametersLoad(f32 altitude_ft, f32 vertical_velocity_fps) {
    control::physics::VerticalState parameters{};
    parameters.altitude_ft = altitude_ft;
    parameters.vertical_velocity_fps = vertical_velocity_fps;
    return parameters;
}

control::BrakeCommand BrakeCommandLoad(f32 extension) {
    control::BrakeCommand parameters{};
    parameters.extension = extension;
    return parameters;
}


PYBIND11_MODULE(ControlBind, m) {
    py::class_<control::MpcParameters>(m, "MpcParameters")
        .def(py::init(&mcpParametersLoad))
        .def_readwrite("extension_step", &control::MpcParameters::extension_step)
        .def_readwrite("prediction_dt", &control::MpcParameters::prediction_dt)
        .def_readwrite("target_apogee_ft", &control::MpcParameters::target_apogee_ft);
    py::class_<control::aero::AeroParameters>(m, "AeroParameters")
        .def(py::init(&aeroParametersLoad))
        .def_readwrite("base_drag_coefficient", &control::aero::AeroParameters::base_drag_coefficient)
        .def_readwrite("max_brake_delta_cd", &control::aero::AeroParameters::max_brake_delta_cd);
    py::class_<control::actuator::ActuatorParameters>(m, "ActuatorParameters")
        .def(py::init(&actuatorParametersLoad))
        .def_readwrite("max_slew_rate", &control::actuator::ActuatorParameters::max_slew_rate);
    py::class_<control::BrakeCommand>(m, "BrakeCommand")
        .def(py::init(&BrakeCommandLoad))
        .def_readwrite("extension", &control::BrakeCommand::extension);
    py::class_<control::physics::RocketParameters>(m, "RocketParameters")
        .def(py::init(&rocketParametersLoad))
        .def_readwrite<>("reference_area_ft2", &control::physics::RocketParameters::reference_area_ft2)
        .def_readwrite<>("mass_slug", &control::physics::RocketParameters::mass_slug);
    py::class_<control::physics::VerticalState>(m, "VerticalState" )
        .def(py::init(&vecticalParametersLoad))
        .def_readwrite<>("altitude_ft", &control::physics::VerticalState::altitude_ft)
        .def_readwrite<>("vertical_velocity_fps", &control::physics::VerticalState::vertical_velocity_fps);

    m.def("slew_mpc", &control::slew_mpc);
}

void ControlBind::test() {
    printf("test\n");
}
