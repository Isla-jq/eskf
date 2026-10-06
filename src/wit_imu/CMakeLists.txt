cmake_minimum_required(VERSION 3.8)
project(wit_imu)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Build the node (wit_imu.cpp + wit_c_sdk.cpp both compiled as C++)
add_executable(wit_imu_node
  src/wit_imu.cpp
  src/wit_c_sdk.cpp)
ament_target_dependencies(wit_imu_node rclcpp sensor_msgs geometry_msgs)
target_include_directories(wit_imu_node PRIVATE include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS wit_imu_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()