#pragma once

#include "sensor_msgs/msg/laser_scan.hpp"
#include "rclcpp/rclcpp.hpp"
#include <Eigen/Dense>
#include "geometry_msgs/msg/pose.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"

using sensor_msgs::msg::LaserScan;
using Eigen::Vector2d;
using geometry_msgs::msg::Pose;
using visualization_msgs::msg::MarkerArray;
using visualization_msgs::msg::Marker;

#define VALID_DIST(d, scan) (std::isfinite(d) && (d) <= scan.range_max && (d) >= scan.range_min)

struct EstimationParams {
  double max_dist_diff;
  double agent_width;
};

LaserScan clean_lidar_scan(LaserScan raw_scan);

MarkerArray create_lidar_markers(const LaserScan& raw_scan, rclcpp::Time time, Pose robot_pose);

/*
 * should pass in the raw lidar scan
 * DONT call clean_lidar_scan before passing it into this function
*/

std::vector<Vector2d> estimate_agent_positions(const LaserScan& scan, const EstimationParams& est_params, Pose robot_pose);

MarkerArray create_est_position_markers(std::vector<Vector2d> positions, rclcpp::Time time);