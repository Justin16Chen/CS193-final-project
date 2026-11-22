#include "crowd_sim/lidar.hpp"
#include "crowd_sim/utils.hpp"
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "crowd_sim/rviz_helper.hpp"

using Eigen::Vector3d;

#define LIDAR_SCAN_HEIGHT (0.4)

LaserScan clean_lidar_scan(LaserScan raw_scan) {
  for (unsigned long int i = 0; i < raw_scan.ranges.size(); i++) {
    double dist = raw_scan.ranges[i];
    if (!std::isfinite(dist) || dist > raw_scan.range_max)
      dist = raw_scan.range_max;
    else if (dist < raw_scan.range_min)
      dist = raw_scan.range_min;
    raw_scan.ranges[i] = dist;
  }
  return raw_scan;
}

MarkerArray create_lidar_markers(const LaserScan& raw_scan, rclcpp::Time time, Pose robot_pose) {
  LaserScan scan = clean_lidar_scan(raw_scan);
  MarkerArray marker_array;
  Vector3d robot_position = Vector3d(robot_pose.position.x, robot_pose.position.y, LIDAR_SCAN_HEIGHT);
  double yaw = tf2::getYaw(robot_pose.orientation);
  double start_angle = yaw + scan.angle_min;

  marker_array.markers.push_back(create_clear_marker("lidar_lines", time));
 
  ColorRGBA color = make_color(0.2, 0.8, 0.2, 1);
  for (long unsigned int i = 0; i < scan.ranges.size(); i++) {
    double angle = start_angle + i * scan.angle_increment;
    Vector3d dir(std::cos(angle), std::sin(angle), 0.);
    double dist = scan.ranges[i];
    Vector3d offset = dir * dist;
    Vector3d end = robot_position + offset;
    Marker marker = create_marker_arrow("lidar_lines", i + 1, time, robot_position, end, color);
    marker_array.markers.push_back(marker);
  }
  return marker_array;
}

void add_est_positions(std::vector<Vector2d>& positions, const LaserScan& scan, const EstimationParams& est_params,
  Pose robot_pose, int start_idx, int i, double total_dist) {
  int indices = i - start_idx;
  double total_angle = indices * scan.angle_increment;
  double avg_dist = total_dist / indices + est_params.agent_width * 0.5;
  double arc_length = total_angle * avg_dist;
  double ratio = arc_length / est_params.agent_width;
  int num_agents_in_blob = (int)(ratio + 0.5);
  double angle_increment = total_angle / (num_agents_in_blob + 1);
  double start_angle = scan.angle_min + start_idx * scan.angle_increment;
  for (int j = 0; j < num_agents_in_blob; j++) {
    double angle = start_angle + angle_increment * (j + 1);
    Vector2d local_pos(cos(angle) * avg_dist, sin(angle) * avg_dist);
    double yaw = tf2::getYaw(robot_pose.orientation);
    Vector2d robot_position = Vector2d(robot_pose.position.x, robot_pose.position.y);
    Vector2d global_pos = Eigen::Rotation2Dd(yaw) * local_pos + robot_position;
    positions.push_back(global_pos);
  }
}

std::vector<Vector2d> estimate_agent_positions(const LaserScan& scan,
  const EstimationParams& est_params, Pose robot_pose) {
  static int num = 0;
  std::vector<Vector2d> positions;
  if (scan.ranges.size() == 0)
    return positions;
  double prev_dist = scan.ranges[0];
  double total_dist = prev_dist;
  int start_idx = -1;
  for (unsigned long int i = 1; i < scan.ranges.size(); i++) {
    double dist = scan.ranges[i];
    if (!VALID_DIST(dist, scan)) {
      if (start_idx != -1) {
        add_est_positions(positions, scan, est_params, robot_pose, start_idx, i, total_dist);
      }
      prev_dist = -1;
      start_idx = -1;
      total_dist = 0;
      continue;
    }
    if (start_idx == -1) {
      prev_dist = dist;
      total_dist = dist;
      start_idx = i;
      continue;
    }
    double delta_dist = ABS(dist - prev_dist);
    double delta_dist_per_angle = delta_dist / scan.angle_increment;
    if (num == 0 && ABS(i - scan.ranges.size() * 0.5) < 10) {
      RCLCPP_INFO(rclcpp::get_logger("est_agent_pos"),
        "dist: %.5lf | delta dist: %.5lf | delta angle: %.5lf | delta dist / delta angle: %.5lf",
        dist, delta_dist, scan.angle_increment, delta_dist_per_angle);
    }
    if (delta_dist_per_angle > est_params.max_dist_diff) {
      add_est_positions(positions, scan, est_params, robot_pose, start_idx, i, total_dist);
      start_idx = i;
      total_dist = dist;
    }
    else {
      total_dist += dist;
    }
    prev_dist = dist;
  }
  num++;
  return positions;
}

MarkerArray create_est_position_markers(std::vector<Vector2d> positions, rclcpp::Time time) {
  MarkerArray markers;
  markers.markers.push_back(create_clear_marker("est_positions", time));
  ColorRGBA color = make_color(0.1, 0.1, 0.9, 0.4);
  double radius = 0.2;
  double height = 1.;
  for (unsigned long int i = 0; i < positions.size(); i++) {
    Vector2d pos = positions[i];
    Vector3d pos3d(pos.x(), pos.y(), height * 0.5);
    markers.markers.push_back(create_marker_cylinder("est_positions", i + 1, time, pos3d, radius, height, color));
  }
  return markers;
}
