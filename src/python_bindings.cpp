#include <pybind11/pybind11.h>

#include "libswervedrive/chassis.h"

namespace py = pybind11;

using namespace swervedrive;

PYBIND11_MODULE(pyswervedrive, m) {
  py::class_<Chassis>(m, "Chassis")
    .def("betas", &Chassis::betas);
}
