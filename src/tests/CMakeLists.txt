SET(COVERAGE OFF CACHE BOOL "Coverage")

find_package(GTest REQUIRED)
include_directories(
  ${GTEST_INCLUDE_DIR}
  ${libswervedrive_SOURCE_DIR}/include
)

file(GLOB TESTSRCS "test*.cpp")

add_executable(run_tests
  ${TESTSRCS}
)

if (COVERAGE)
    target_compile_options(run_tests PRIVATE --coverage)
    target_link_libraries(run_tests PRIVATE --coverage)
endif()

target_link_libraries(run_tests
  gtest
  gtest_main
  pthread
  swervedrive
)

add_test(all_tests run_tests)
