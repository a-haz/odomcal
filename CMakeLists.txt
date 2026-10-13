cmake_minimum_required(VERSION 3.16)
project(odomcal LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(POLICY CMP0135)
    cmake_policy(SET CMP0135 NEW)
endif()

# The repository root builds the benchmark and the unit tests. A downstream
# package can add_subdirectory() this tree and link the odomcal target.
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    set(_odomcal_is_toplevel ON)
else()
    set(_odomcal_is_toplevel OFF)
endif()
option(ODOM_CAL_BUILD_TESTS "Build the odomcal unit tests" ${_odomcal_is_toplevel})
option(ODOM_CAL_BUILD_BENCH "Build the odomcal_bench executable" ${_odomcal_is_toplevel})

include(FetchContent)

# Header-only Eigen. Skip Eigen's own project so the build does not compile its tests.
if(CMAKE_VERSION VERSION_GREATER_EQUAL "3.30")
    cmake_policy(SET CMP0169 OLD)
endif()
FetchContent_Declare(
    eigen
    URL https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)
FetchContent_GetProperties(eigen)
if(NOT eigen_POPULATED)
    FetchContent_Populate(eigen)
endif()
add_library(eigen INTERFACE)
add_library(Eigen3::Eigen ALIAS eigen)
target_include_directories(eigen SYSTEM INTERFACE ${eigen_SOURCE_DIR})

if(ODOM_CAL_BUILD_TESTS)
    set(BUILD_GMOCK OFF CACHE BOOL "" FORCE)
    set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
    FetchContent_Declare(
        googletest
        URL https://github.com/google/googletest/archive/refs/tags/v1.15.2.tar.gz
        DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    )
    FetchContent_MakeAvailable(googletest)
endif()

add_library(odomcal
    src/diff_drive.cpp
    src/simulator.cpp
    src/umbmark.cpp
    src/least_squares.cpp
    src/benchmark.cpp
)
target_include_directories(odomcal PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(odomcal PUBLIC Eigen3::Eigen)
target_compile_options(odomcal PRIVATE -Wall -Wextra -Wpedantic -Werror)

if(ODOM_CAL_BUILD_BENCH)
    add_executable(odomcal_bench src/bench_main.cpp)
    target_link_libraries(odomcal_bench PRIVATE odomcal)
    target_compile_options(odomcal_bench PRIVATE -Wall -Wextra -Wpedantic -Werror)
endif()

if(ODOM_CAL_BUILD_TESTS)
    enable_testing()
    add_executable(odomcal_tests
        tests/test_kinematics.cpp
        tests/test_errors.cpp
        tests/test_umbmark.cpp
        tests/test_least_squares.cpp
        tests/test_determinism.cpp
    )
    target_include_directories(odomcal_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/tests)
    target_link_libraries(odomcal_tests PRIVATE odomcal GTest::gtest_main)
    target_compile_options(odomcal_tests PRIVATE -Wall -Wextra -Wpedantic -Werror)

    include(GoogleTest)
    gtest_discover_tests(odomcal_tests DISCOVERY_MODE PRE_TEST)
endif()
