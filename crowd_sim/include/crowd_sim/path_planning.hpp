#pragma once
#include "rclcpp/rclcpp.hpp"
#include <Eigen/Dense>
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "crowd_interfaces/msg/cost_grid.hpp"
#include "crowd_sim/utils.hpp"
#include "crowd_sim/planning_math.hpp"

using Eigen::Vector2d;
using visualization_msgs::msg::MarkerArray;
using geometry_msgs::msg::Twist;
using geometry_msgs::msg::Pose;
using crowd_interfaces::msg::CostGrid;

vec2d_list estimate_velocities(vec2d_list old_positions,
  vec2d_list new_positions, double dt, double d_max = 0.5);


struct SmoothParams {
  std::vector<double> weights;
};

// Weighted average of the most recent velocity estimates, per agent.
// prev_velocities: one vec2d_list per past frame, oldest first, newest last.
// weights: one weight per frame, aligned so the last weight applies to the newest frame.
// Agents are indexed by the newest frame; older frames with fewer agents only contribute where defined.
vec2d_list smooth_velocities(const std::vector<vec2d_list> & prev_velocities,
  const SmoothParams& smooth_params);

MarkerArray create_velocity_markers(vec2d_list positions,
  vec2d_list velocities, rclcpp::Time time);

// Result of one planning cycle: the chosen command plus the full cost grid it was
// picked from, so the grid can be published for offline tuning (scripts/plot_costs.py).
// grid.header is left unset; the caller stamps it.
struct PlanResult {
  Twist cmd;
  CostGrid grid;
};

PlanResult calculate_output(const vec2d_list& positions, const vec2d_list& velocities,
  PredictParams predict_params, Pose robot_pose, Vector2d goal_position, Twist prev_output);
