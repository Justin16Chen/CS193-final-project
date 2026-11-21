#pragma once

#include "rclcpp/time.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <Eigen/Dense>

visualization_msgs::msg::Marker create_marker_cylinder(int id, rclcpp::Time time, 
  const Eigen::Vector3d& pos, double radius, double height, double r, double g, double b);

visualization_msgs::msg::Marker create_marker_arrow(int id, rclcpp::Time time,
  const Eigen::Vector3d& from, const Eigen::Vector3d& to, double r, double g, double b);