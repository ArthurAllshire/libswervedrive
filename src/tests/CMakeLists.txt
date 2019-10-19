include(FetchContent)

FetchContent_Declare(
  googletest
  GIT_REPOSITORY https://github.com/google/googletest.git
  GIT_TAG        release-1.10.0
)

FetchContent_GetProperties(googletest)
if(NOT googletest_POPULATED)
  FetchContent_Populate(googletest)
  add_subdirectory(${googletest_SOURCE_DIR} ${googletest_BINARY_DIR})
endif()

include_directories(
  ${GTEST_INCLUDE_DIR}
  ${libswervedrive_SOURCE_DIR}/include
)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

file(GLOB TESTSRCS "test*.cpp")

add_executable(run_tests
  ${TESTSRCS}
)

target_link_libraries(run_tests PRIVATE --coverage
  gtest
  gtest_main
  pthread
  swervedrive
  Eigen3::Eigen
)

add_test(all_tests run_tests)
