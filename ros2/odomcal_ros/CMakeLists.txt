cmake_minimum_required(VERSION 3.16)
project(odomcal_ros)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Link the repository library in place. Tests and the benchmark stay with the
# standalone build so this package does not fetch GoogleTest for them.
set(ODOM_CAL_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(ODOM_CAL_BUILD_BENCH OFF CACHE BOOL "" FORCE)
set(_odomcal_ros_build_shared_libs "${BUILD_SHARED_LIBS}")
set(BUILD_SHARED_LIBS OFF)
get_filename_component(ODOM_CAL_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
add_subdirectory(${ODOM_CAL_ROOT} ${CMAKE_CURRENT_BINARY_DIR}/odomcal_core)
set(BUILD_SHARED_LIBS "${_odomcal_ros_build_shared_libs}")

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetCalibration.srv"
)

add_library(odom_calibrator_core STATIC
  src/odom_calibrator.cpp
)
target_include_directories(odom_calibrator_core PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
)
target_compile_features(odom_calibrator_core PUBLIC cxx_std_17)
target_link_libraries(odom_calibrator_core PUBLIC odomcal)
ament_target_dependencies(odom_calibrator_core SYSTEM PUBLIC
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2_ros
  std_srvs
)
target_compile_options(odom_calibrator_core PRIVATE -Wall -Wextra -Wpedantic -Werror)

rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(odom_calibrator_core PUBLIC "${cpp_typesupport_target}")

add_executable(odomcal_node src/odomcal_node.cpp)
target_link_libraries(odomcal_node PRIVATE odom_calibrator_core)
target_compile_options(odomcal_node PRIVATE -Wall -Wextra -Wpedantic -Werror)

add_executable(replay_square src/replay_square.cpp)
target_link_libraries(replay_square PRIVATE odom_calibrator_core)
target_compile_options(replay_square PRIVATE -Wall -Wextra -Wpedantic -Werror)

install(TARGETS odomcal_node replay_square
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_odom_calibrator test/test_odom_calibrator.cpp)
  target_link_libraries(test_odom_calibrator odom_calibrator_core)
  target_compile_options(test_odom_calibrator PRIVATE -Wall -Wextra -Wpedantic -Werror)
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
