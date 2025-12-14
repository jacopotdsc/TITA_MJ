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