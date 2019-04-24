#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <Eigen/Dense>

#include <sstream> // used for representation
#include <vector>

#include "libswervedrive/chassis.h"
#include "libswervedrive/estimator.h"

namespace py = pybind11;

using namespace swervedrive;
using Eigen::VectorXd;

PYBIND11_MODULE(pyswervedrive, m) {
  py::class_<Chassis>(m, "Chassis")
    .def(py::init<VectorXd, // alpha
                  VectorXd, // l
                  VectorXd, // b
                  VectorXd, // r
                  Bounds,   // beta_bounds
                  Bounds,   // beta_dot_bounds
                  Bounds,   // beta_2dot_bounds
                  Bounds,   // phi_dot_bounds
                  Bounds    // phi_2dot_bounds
                  >())
    .def("betas", &Chassis::betas)
    .def("__repr__",
         [](const Chassis& c) {
           std::stringstream ss;
           ss << "Chassis Object\n"
              << "Alpha:\n" << c.alpha_
              << "\nl:\n" << c.l_vector_ 
              << "\nb:\n" << c.b_
              << "\nr:\n" << c.r_
              ;
           // TODO: Add more parameters, prettify printing
           return ss.str();
         }
    );

  py::class_<Estimator>(m, "Estimator")
      .def(py::init<const Chassis &,
                    Epsilon,
                    double,
                    double,
                    double,
                    double,
                    double
      >(),
                    py::arg("chassis"),
                    py::arg("epsilon")=Eigen::VectorXd::Zero(3, 1),
                    py::arg("eta_lambda")=1e-4,
                    py::arg("eta_delta")=1e-2,
                    py::arg("min_delta_line_search")=1e-2,
                    py::arg("max_iter_lambda")=50,
                    py::arg("singularity_tolerance")=1e-3)
      .def("estimate", &Estimator::estimate,
           "Estimate the current ICR given wheel steering angles");
}
