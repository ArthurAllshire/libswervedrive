# Download and unpack googletest at configure time
configure_file(CMakeLists.txt.in googletest-download/CMakeLists.txt)
execute_process(COMMAND ${CMAKE_COMMAND} -G "${CMAKE_GENERATOR}" .
  RESULT_VARIABLE result
  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/googletest-download )
if(result)
  message(FATAL_ERROR "CMake step for googletest failed: ${result}")
endif()
execute_process(COMMAND ${CMAKE_COMMAND} --build .
  RESULT_VARIABLE result
  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/googletest-download )
if(result)
  message(FATAL_ERROR "Build step for googletest failed: ${result}")
endif()

# Prevent overriding the parent project's compiler/linker
# settings on Windows
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

# Add googletest directly to our build. This defines
# the gtest and gtest_main targets.
add_subdirectory(${CMAKE_CURRENT_BINARY_DIR}/googletest-src
                 ${CMAKE_CURRENT_BINARY_DIR}/googletest-build
                 EXCLUDE_FROM_ALL)

# The gtest/gtest_main targets carry header search path
# dependencies automatically when using CMake 2.8.11 or
# later. Otherwise we have to add them here ourselves.
if (CMAKE_VERSION VERSION_LESS 2.8.11)
  include_directories("${gtest_SOURCE_DIR}/include")
endif()

include_directories(
  ${GTEST_INCLUDE_DIR}
  ${libswervedrive_SOURCE_DIR}/include
)

file(GLOB TESTSRCS "test*.cpp")

add_executable(run_tests
  ${TESTSRCS}
)
if (Eigen3_VERSION VERSION_EQUAL "3.3" OR Eigen3_VERSION VERSION_GREATER "3.3")
  # If we have a version of Eigen before 3.3 then the next line is not needed
  # Travis runs Ubuntu 16.04 which has Eigen3.3~beta1-2 which fails if the next line
  # is included
else()
  include_directories ( ${EIGEN3_INCLUDE_DIRS} )
endif()

target_link_libraries(run_tests PRIVATE --coverage
  gtest
  gtest_main
  pthread
  swervedrive
)

add_test(all_tests run_tests)
