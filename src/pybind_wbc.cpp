#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include "MPC.hpp"
#include "WholeBodyController.hpp"
#include "RobotState.hpp"
#include "utils.hpp"

#include <blasfeo_target.h>
#include <hpipm_d_dense_qp_ipm.h>



namespace py = pybind11;

PYBIND11_MODULE(wbc, m) {
  m.doc() = "pybind11 bindings for labrob::WholeBodyController";

  py::class_<labrob::WholeBodyControllerParams>(m, "WholeBodyControllerParams")
    .def(py::init<>())
    .def_readwrite("Kp_motion", &labrob::WholeBodyControllerParams::Kp_motion)
    .def_readwrite("Kd_motion", &labrob::WholeBodyControllerParams::Kd_motion)
    .def_readwrite("Kp_regulation", &labrob::WholeBodyControllerParams::Kp_regulation)
    .def_readwrite("Kd_regulation", &labrob::WholeBodyControllerParams::Kd_regulation)
    .def_readwrite("Kp_wheel", &labrob::WholeBodyControllerParams::Kp_wheel)
    .def_readwrite("Kd_wheel", &labrob::WholeBodyControllerParams::Kd_wheel)
    .def_readwrite("weight_q_ddot", &labrob::WholeBodyControllerParams::weight_q_ddot)
    .def_readwrite("weight_com", &labrob::WholeBodyControllerParams::weight_com)
    .def_readwrite("weight_lwheel", &labrob::WholeBodyControllerParams::weight_lwheel)
    .def_readwrite("weight_rwheel", &labrob::WholeBodyControllerParams::weight_rwheel)
    .def_readwrite("weight_base", &labrob::WholeBodyControllerParams::weight_base)
    .def_readwrite("weight_regulation", &labrob::WholeBodyControllerParams::weight_regulation)
    .def_readwrite("weight_angular_momentum", &labrob::WholeBodyControllerParams::weight_angular_momentum)
    .def_readwrite("cmm_selection_matrix_x", &labrob::WholeBodyControllerParams::cmm_selection_matrix_x)
    .def_readwrite("cmm_selection_matrix_y", &labrob::WholeBodyControllerParams::cmm_selection_matrix_y)
    .def_readwrite("cmm_selection_matrix_z", &labrob::WholeBodyControllerParams::cmm_selection_matrix_z)
    .def_readwrite("gamma", &labrob::WholeBodyControllerParams::gamma)
    .def_readwrite("mu", &labrob::WholeBodyControllerParams::mu)
    .def_static("getDefaultParams", &labrob::WholeBodyControllerParams::getDefaultParams);


  py::class_<labrob::RobotState>(m, "RobotState")
        .def(py::init<>())
        .def_readwrite("position", &labrob::RobotState::position)
        .def_readwrite("orientation", &labrob::RobotState::orientation)
        .def_readwrite("linear_velocity", &labrob::RobotState::linear_velocity)
        .def_readwrite("angular_velocity", &labrob::RobotState::angular_velocity)
        .def_readwrite("joint_state", &labrob::RobotState::joint_state)
        .def_readwrite("total_force", &labrob::RobotState::total_force)
        .def_readwrite("contact_points", &labrob::RobotState::contact_points)
        .def_readwrite("contact_forces", &labrob::RobotState::contact_forces);

   py::class_<labrob::DesiredConfiguration>(m, "DesiredConfiguration")
        .def(py::init<>())
        .def_readwrite("position", &labrob::DesiredConfiguration::position)
        .def_readwrite("orientation", &labrob::DesiredConfiguration::orientation)
        .def_readwrite("linear_velocity", &labrob::DesiredConfiguration::linear_velocity)
        .def_readwrite("angular_velocity", &labrob::DesiredConfiguration::angular_velocity)
        .def_readwrite("qjnt", &labrob::DesiredConfiguration::qjnt)
        .def_readwrite("qjntdot", &labrob::DesiredConfiguration::qjntdot)
        .def_readwrite("qjntddot", &labrob::DesiredConfiguration::qjntddot)
        .def_readwrite("com", &labrob::DesiredConfiguration::com)
        .def_readwrite("lwheel", &labrob::DesiredConfiguration::lwheel)
        .def_readwrite("rwheel", &labrob::DesiredConfiguration::rwheel)
        .def_readwrite("base_link", &labrob::DesiredConfiguration::base_link)
        .def_readwrite("in_contact", &labrob::DesiredConfiguration::in_contact);
      
  py::class_<labrob::WholeBodyController>(m, "WholeBodyController")
    
    .def(py::init([](const  labrob::WholeBodyControllerParams &params,
                      const pinocchio::Model &robot_model,
                      const labrob::RobotState &initial_robot_state,
                      double sample_time,
                      py::dict armature_dict) {

            std::map<std::string,double> armature;
            for (auto item : armature_dict) {
                armature[item.first.cast<std::string>()] = item.second.cast<double>();
            }

            return labrob::WholeBodyController(params, robot_model, initial_robot_state, sample_time, armature);
        }),
        py::arg("params"),
        py::arg("robot_model"),
        py::arg("initial_robot_state"),
        py::arg("sample_time"),
        py::arg("armature")
      )

    .def("compute_inverse_dynamics", [](labrob::WholeBodyController &self,
              const labrob::RobotState &robot_state,
              const labrob::DesiredConfiguration &des_configuration) {
              return self.compute_inverse_dynamics(robot_state, des_configuration);
          },
          py::arg("robot_state"), 
          py::arg("des_configuration")
        );
}