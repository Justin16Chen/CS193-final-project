#pragma once

#include "rclcpp/time.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <cstdint>
#include <Eigen/Dense>

visualization_msgs::msg::Marker create_marker_cylinder(int id, rclcpp::Time time, const Eigen::Vector2d& pos, double radius, double height);

visualization_msgs::msg::Marker create_marker_arrow(int id, rclcpp::Time time, const Eigen::Vector2d& from, const Eigen::Vector2d& to);