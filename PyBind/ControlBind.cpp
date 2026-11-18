
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include "control/brake_command.hpp"
#include "control/actuator/actuator_model.hpp"
#include "control/controller/ideal_mpc.hpp"
#include "control/controller/slew_mpc.hpp"

namespace py = pybind11;

#include "ControlBind.h"




PYBIND11_MODULE(ControlBind, m) {
    py::class_<control::MpcParameters>(m, "MpcParameters")
        .def(py::init<>())
        .def_readwrite("extension_step", &control::MpcParameters::extension_step)
        .def_readwrite("prediction_dt", &control::MpcParameters::prediction_dt)
        .def_readwrite("target_apogee_ft", &control::MpcParameters::target_apogee_ft);
    py::class_<control::aero::AeroParameters>(m, "AeroParameters")
        .def(py::init<>())
        .def_readwrite("base_drag_coefficient", &control::aero::AeroParameters::base_drag_coefficient)
        .def_readwrite("max_brake_delta_cd", &control::aero::AeroParameters::max_brake_delta_cd);
    py::class_<control::actuator::ActuatorParameters>(m, "ActuatorParameters")
        .def(py::init<>())
        .def_readwrite("base_drag_coefficient", &control::actuator::ActuatorParameters::max_slew_rate);
    py::class_<control::BrakeCommand>(m, "BrakeCommand")
        .def(py::init<>())
        .def_readwrite("extension", &control::BrakeCommand::extension);
    py::class_<control::physics::RocketParameters>(m, "RocketParameters")
        .def(py::init<>())
        .def_readwrite<>("reference_area_ft2", &control::physics::RocketParameters::reference_area_ft2)
        .def_readwrite<>("mass_slug", &control::physics::RocketParameters::mass_slug);
    py::class_<control::physics::VerticalState>(m, "VerticalState" )
        .def(py::init<>())
        .def_readwrite<>("altitude_ft", &control::physics::VerticalState::altitude_ft)
        .def_readwrite<>("vertical_velocity_fps", &control::physics::VerticalState::vertical_velocity_fps);

    m.def("slew_mpc", &control::slew_mpc);
}

void ControlBind::test() {
    printf("test\n");
}
