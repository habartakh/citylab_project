cmake_minimum_required(VERSION 3.8)
project(robot_patrol)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)


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

add_executable(robot_patrol_node src/patrol.cpp)
ament_target_dependencies(robot_patrol_node rclcpp nav_msgs sensor_msgs geometry_msgs)

add_executable(direction_service_node src/direction_service.cpp)
ament_target_dependencies(direction_service_node rclcpp nav_msgs sensor_msgs)

add_executable(test_service_node src/test_service.cpp)
ament_target_dependencies(test_service_node rclcpp nav_msgs sensor_msgs)

add_executable(patrol_with_service_node src/patrol_with_service.cpp)
ament_target_dependencies(patrol_with_service_node rclcpp nav_msgs sensor_msgs)

install(TARGETS
   robot_patrol_node
   direction_service_node
   test_service_node
   patrol_with_service_node
   DESTINATION lib/${PROJECT_NAME}
)

# Install launch files.
install(DIRECTORY
  launch
  config # without it config file is not found during launch
  DESTINATION share/${PROJECT_NAME}/
)

# add custom interface
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetDirection.srv"
  DEPENDENCIES sensor_msgs
)

rosidl_target_interfaces(direction_service_node 
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

rosidl_target_interfaces(test_service_node 
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

rosidl_target_interfaces(patrol_with_service_node 
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

ament_package()
