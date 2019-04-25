cmake_minimum_required (VERSION 3.0)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)
set(PYBIND11_CPP_STANDARD -std=c++17)
set(PYBIND11_PYTHON_VERSION 3.6)
add_subdirectory(pybind11)

include_directories (${libswervedrive_SOURCE_DIR}/include)
add_library (swervedrive
  chassis.cpp
  controller.cpp
  estimator.cpp
  kinematic_model.cpp
  time_scaler.cpp
)

target_link_libraries (swervedrive
  Eigen3::Eigen
)

add_library(pyswervedrive MODULE python_bindings.cpp)
target_link_libraries (pyswervedrive PRIVATE pybind11::module
  swervedrive
)
set_target_properties(pyswervedrive PROPERTIES PREFIX "${PYTHON_MODULE_PREFIX}"
  SUFFIX "${PYTHON_MODULE_EXTENSION}")
