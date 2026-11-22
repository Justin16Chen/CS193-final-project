#include "crowd_sim/rviz_helper.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"

using geometry_msgs::msg::Point;

ColorRGBA make_color(double r, double g, double b, double a) {
  ColorRGBA color;
  color.r = r;
  color.g = g;
  color.b = b;
  color.a = a;
  return color;
}

Marker create_clear_marker(std::string name, rclcpp::Time time) {
  Marker clear;
  clear.header.frame_id = "odom";
  clear.header.stamp = time;
  clear.ns = name;
  clear.id = 0;
  clear.action = Marker::DELETEALL;
  return clear;
}

Marker create_marker_cylinder(std::string name, int id, rclcpp::Time time, const Eigen::Vector3d& pos,
  double radius, double height, ColorRGBA color) {
  Marker m;
  m.header.frame_id = "odom";
  m.header.stamp = time;
  m.ns = name;
  m.id = id;
  m.type = Marker::CYLINDER;
  m.action = Marker::ADD;
  m.pose.position.x = pos.x();
  m.pose.position.y = pos.y();
  m.pose.position.z = pos.z();
  m.pose.orientation.w = 1.0;
  m.scale.x = radius * 2;
  m.scale.y = radius * 2;
  m.scale.z = height;
  m.color = color;
  return m;
}

Marker create_marker_arrow(std::string name, int id, rclcpp::Time time, const Eigen::Vector3d& from,
  const Eigen::Vector3d& to, ColorRGBA color) {
  Marker m;
  m.header.frame_id = "odom";
  m.header.stamp = time;
  m.ns = name;
  m.id = id;                            // same ns+id overwrites the old arrow
  m.type = Marker::ARROW;
  m.action = Marker::ADD;

  Point p0, p1;
  p0.x = from.x(); p0.y = from.y(); p0.z = from.z();
  p1.x = to.x();   p1.y = to.y();   p1.z = to.z();
  m.points = {p0, p1};

  m.scale.x = 0.05;   // shaft diameter
  m.scale.y = 0.1;    // head diameter
  m.scale.z = 0.1;    // head length

  m.color = color;

  m.pose.orientation.w = 1.0;  // avoids an invalid-quaternion warning
  return m;
}