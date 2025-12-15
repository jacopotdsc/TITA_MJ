#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>

#include "WalkingManager.hpp"
#include "MPC.hpp"
#include "JointState.hpp"
#include "RobotState.hpp"
#include "JointCommand.hpp"

namespace py = pybind11;
using namespace labrob;

PYBIND11_MODULE(wm, m) {
    m.doc() = "Python bindings for WalkingManager";

    py::class_<JointData>(m, "JointData")
        .def(py::init<>())
        .def_readwrite("pos", &JointData::pos)
        .def_readwrite("vel", &JointData::vel)
        .def_readwrite("acc", &JointData::acc)
        .def_readwrite("eff", &JointData::eff);
    
    py::class_<JointState>(m, "JointState")
        .def(py::init<>())
        .def("__getitem__", [](JointState &js, const std::string &key) {
            return js[key]; 
        })
        .def("__setitem__", [](JointState &js, const std::string &key, const JointData &val) {
            js[key] = val;
        })
        .def("__iter__", [](JointState &js) {
            return py::make_iterator(js.begin(), js.end());
        }, py::keep_alive<0, 1>());

    py::class_<RobotState>(m, "RobotState")
        .def(py::init<>())
        .def_readwrite("position", &RobotState::position)
        .def_property("orientation",
            [](const RobotState &r) { 
                return r.orientation.coeffs(); 
            },
            [](RobotState &r, const Eigen::Vector4d &v) {
                r.orientation.coeffs() = v;   
            }
        )
        .def_readwrite("linear_velocity", &RobotState::linear_velocity)
        .def_readwrite("angular_velocity", &RobotState::angular_velocity)
        .def_readwrite("joint_state", &RobotState::joint_state)
        .def_readwrite("total_force", &RobotState::total_force)
        .def_readwrite("contact_points", &RobotState::contact_points)
        .def_readwrite("contact_forces", &RobotState::contact_forces);

    py::class_<JointCommand>(m, "JointCommand")
        .def(py::init<>())
        .def("__getitem__", [](const JointCommand &jc, const std::string &key) {
            return jc[key];
        })
        .def("__setitem__", [](JointCommand &jc, const std::string &key, double value) {
            jc[key] = value;
        })
        .def("__iter__", [](JointCommand &jc) {
            return py::make_iterator(jc.begin(), jc.end());
        }, py::keep_alive<0, 1>());

    py::class_<WalkingManager>(m, "WalkingManager")
        .def(py::init<>())
        .def("init", [](WalkingManager &wm, const RobotState &state, py::dict &armatures) {
            // Convert py::dict to std::map
            std::map<std::string, double> armatures_map;
            for (auto item : armatures) {
                armatures_map[item.first.cast<std::string>()] = item.second.cast<double>();
            }
            return wm.init(state, armatures_map);
        })
        .def("update", [](WalkingManager &wm, const RobotState &state, JointCommand &cmd) {
            wm.update(state, cmd);
        });
}
