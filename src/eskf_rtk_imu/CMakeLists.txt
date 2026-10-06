cmake_minimum_required(VERSION 3.8)
project(eskf_rtk_imu)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
list(APPEND CMAKE_MODULE_PATH "/usr/share/cmake/geographiclib")
find_package(GeographicLib REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(eskf_rtk_imu_node src/main.cpp)
ament_target_dependencies(eskf_rtk_imu_node
  rclcpp
  sensor_msgs
  nav_msgs
  std_msgs
)
target_link_libraries(eskf_rtk_imu_node ${GeographicLib_LIBRARIES})
target_include_directories(eskf_rtk_imu_node
  PRIVATE
  ${GeographicLib_INCLUDE_DIRS}
  ${CMAKE_CURRENT_LIST_DIR}/include)

target_sources(eskf_rtk_imu_node
  PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}/src/gps_processor.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/imu_processor.cpp
)

target_link_libraries(eskf_rtk_imu_node Eigen3::Eigen)
install(TARGETS eskf_rtk_imu_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)

  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
