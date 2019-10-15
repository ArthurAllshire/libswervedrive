#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <Eigen/Dense>

#include <sstream>  // used for representation
#include <vector>

#include "libswervedrive/chassis.h"
#include "libswervedrive/controller.h"
#include "libswervedrive/estimator.h"
#include "libswervedrive/kinematic_model.h"
#include "libswervedrive/time_scaler.h"

namespace py = pybind11;

using namespace swervedrive;
using Eigen::VectorXd;
using std::vector;

PYBIND11_MODULE(pyswervedrive, m)
{
  py::class_<Chassis> chassis(m, "Chassis");
  chassis
      .def(py::init<VectorXd,  // alpha
                    VectorXd,  // l
                    VectorXd   // r
                    >())
      .def(py::init<VectorXd,  // alpha
                    VectorXd,  // l
                    VectorXd,  // r
                    VectorXd   // b
                    >())
      .def("betas", &Chassis::betas)
      .def_readwrite("xi", &Chassis::xi_)
      .def_readonly("state", &Chassis::state_)
      .def("__repr__", [](const Chassis& c) {
        std::stringstream ss;
        ss << "Chassis Object\n"
           << "Alpha:\n"
           << c.alpha_ << "\nl:\n"
           << c.l_vector_ << "\nr:\n"
           << c.r_ << "\nb:\n"
           << c.b_;
        // TODO: Add more parameters, prettify printing
        return ss.str();
      });
  py::enum_<Chassis::State>(chassis, "State")
      .value("Stopping", Chassis::STOPPING)
      .value("Reconfiguring", Chassis::RECONFIGURING)
      .value("Running", Chassis::RUNNING)
      .export_values();

  py::class_<Controller>(m, "Controller")
      .def(py::init<const Chassis&>(), py::arg("chassis"))
      .def("controlStep", &Controller::controlStep,
           "Return pairs of beta and phi_dot for desired x_dot, y_dot and theta_dot")
      .def("updateStates", py::overload_cast<const VectorXd&, const VectorXd&>(&Controller::updateStates),
           "Update the observed beta and phi_dot states")
      .def("updateStates", py::overload_cast<const vector<ModuleState>&>(&Controller::updateStates),
           "Update the observed beta and phi_dot states");

  py::class_<Estimator>(m, "Estimator")
      .def(py::init<const Chassis&, Xi, double, double, double, double, double>(), py::arg("chassis"),
           py::arg("Xi") = Eigen::VectorXd::Zero(3, 1), py::arg("eta_lambda") = 1e-4, py::arg("eta_delta") = 1e-2,
           py::arg("min_delta_line_search") = 1e-2, py::arg("max_iter_lambda") = 50,
           py::arg("singularity_tolerance") = 1e-3)
      .def("estimate", &Estimator::estimate, "Estimate the current ICR given wheel steering angles");
}
