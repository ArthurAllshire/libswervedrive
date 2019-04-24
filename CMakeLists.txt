cmake_minimum_required (VERSION 3.0)

project (libswervedrive)

set (CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror -Wall -fPIC")

enable_testing()

add_subdirectory(src bin)
add_subdirectory(src/tests bin/tests)
