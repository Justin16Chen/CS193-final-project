cmake_minimum_required(VERSION 3.8)
project(crowd_sim)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
option(CROWD_SIM_ASAN "Build with AddressSanitizer" OFF)
if (CROWD_SIM_ASAN)
  add_compile_options(-fsanitize=address -fsanitize=undefined -fno-omit-frame-pointer -g -O1)
  add_link_options(-fsanitize=address -fsanitize=undefined)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(crowd_interfaces REQUIRED)
find_package(ros_gz_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

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

  # Pure-math unit tests: no ROS link dependencies, only Eigen.
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_planning_math test/test_planning_math.cpp src/planning_math.cpp)
  target_include_directories(test_planning_math PRIVATE include)
  target_link_libraries(test_planning_math Eigen3::Eigen)
endif()

add_executable(controller src/controller.cpp src/rviz_helper.cpp src/lidar.cpp src/path_planning.cpp src/planning_math.cpp)
ament_target_dependencies(controller
  rclcpp
  sensor_msgs geometry_msgs visualization_msgs nav_msgs geometry_msgs
  tf2 tf2_geometry_msgs)
target_link_libraries(controller Eigen3::Eigen)
target_include_directories(controller PRIVATE include)

add_executable(crowd src/crowd.cpp src/agent.cpp src/rviz_helper.cpp)
ament_target_dependencies(crowd 
  rclcpp
  crowd_interfaces ros_gz_interfaces
  visualization_msgs nav_msgs)
target_link_libraries(crowd Eigen3::Eigen)
target_include_directories(crowd PRIVATE include)

install(TARGETS controller crowd DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config gazebo_sim rviz DESTINATION share/${PROJECT_NAME})

ament_package()
