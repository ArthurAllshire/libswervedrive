cmake_minimum_required (VERSION 3.0)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

include_directories (${libswervedrive_SOURCE_DIR}/include)
add_library (swervedrive controller.cpp estimator.cpp)
target_link_libraries (swervedrive Eigen3::Eigen)

