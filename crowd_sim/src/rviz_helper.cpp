#include "crowd_sim/rviz_helper.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"

using visualization_msgs::msg::Marker;
using geometry_msgs::msg::Point;

Marker create_marker_cylinder(int id, rclcpp::Time time, const Eigen::Vector3d& pos,
  double radius, double height, double r, double g, double b) {
  Marker m;
  m.header.frame_id = "odom";
  m.header.stamp = time;
  m.ns = "people";
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
  m.color.r = r;
  m.color.g = g;
  m.color.b = b;
  m.color.a = 1;
  return m;
}

Marker create_marker_arrow(int id, rclcpp::Time time, const Eigen::Vector3d& from,
  const Eigen::Vector3d& to, double r, double g, double b) {
  Marker m;
  m.header.frame_id = "odom";
  m.header.stamp = time;
  m.ns = "arrows";
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

  m.color.r = r; m.color.g = g; m.color.b = b;
  m.color.a = 1.0;    // if alpha is 0, the arrow is invisible

  m.pose.orientation.w = 1.0;  // avoids an invalid-quaternion warning
  return m;
}