#pragma once

#include "rclcpp/time.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <Eigen/Dense>
#include "std_msgs/msg/color_rgba.hpp"

using visualization_msgs::msg::Marker;
using std_msgs::msg::ColorRGBA;

ColorRGBA make_color(double r, double g, double b, double a);

Marker create_clear_marker(std::string name, rclcpp::Time);

Marker create_marker_cylinder(std::string name, int id, rclcpp::Time time, 
  const Eigen::Vector3d& pos, double radius, double height, ColorRGBA color);

Marker create_marker_arrow(std::string name, int id, rclcpp::Time time,
  const Eigen::Vector3d& from, const Eigen::Vector3d& to, ColorRGBA color);