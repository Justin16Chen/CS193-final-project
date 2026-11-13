cmake_minimum_required(VERSION 3.8)
project(cs193project)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(crowd_interfaces REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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

add_executable(clock_reciever src/clock_reciever.cpp)
ament_target_dependencies(clock_reciever rclcpp rosgraph_msgs)

add_executable(controller src/controller.cpp)
ament_target_dependencies(controller rclcpp sensor_msgs geometry_msgs)

add_executable(crowd src/crowd.cpp)
ament_target_dependencies(crowd rclcpp crowd_interfaces)

install(TARGETS clock_reciever controller crowd DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch gazebo_sim DESTINATION share/${PROJECT_NAME})

ament_package()
