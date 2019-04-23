cmake_minimum_required (VERSION 3.0)

find_package (Eigen3 REQUIRED)
#find_package (Eigen3 3.3 REQUIRED NO_MODULE)

include_directories (${libswervedrive_SOURCE_DIR}/include)
add_library (swervedrive
  chassis.cpp
  controller.cpp
  estimator.cpp)

if (Eigen3_VERSION VERSION_EQUAL "3.3" OR Eigen3_VERSION VERSION_GREATER "3.3")
  # If we have a version of Eigen before 3.3 then the next line is not needed
  # Travis runs Ubuntu 16.04 which has Eigen3.3~beta1-2 which fails if the next line
  # is included
  target_link_libraries (swervedrive Eigen3::Eigen)
else()
  include_directories ( ${EIGEN3_INCLUDE_DIRS} )
endif()
