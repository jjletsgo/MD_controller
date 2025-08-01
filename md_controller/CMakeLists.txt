cmake_minimum_required(VERSION 3.8)
project(md_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(serial REQUIRED)

include_directories(
  include
  ${serial_INCLUDE_DIRS}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# 기존 md_controller 노드
add_executable(md_controller src/md_controller.cpp src/com.cpp)
ament_target_dependencies(
  md_controller
  rclcpp std_msgs serial geometry_msgs tf2_ros tf2 sensor_msgs
)
install(TARGETS
  md_controller
  DESTINATION lib/${PROJECT_NAME}
)

# ──────────────────────────────────────────────
# 새로 추가하는 cmd_vel_to_rpm 노드
add_executable(cmd_vel_to_rpm
  src/cmd_vel_to_rpm.cpp
  src/main.cpp
)
ament_target_dependencies(
  cmd_vel_to_rpm
  rclcpp geometry_msgs std_msgs
)


install(TARGETS
  cmd_vel_to_rpm
  DESTINATION lib/${PROJECT_NAME}
)
# ──────────────────────────────────────────────

install(
  DIRECTORY launch rviz include
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
