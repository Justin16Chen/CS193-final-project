#include "rclcpp/rclcpp.hpp"
#include <Eigen/Dense>
#include "visualization_msgs/msg/marker_array.hpp"
#include "crowd_sim/utils.hpp"

using Eigen::Vector2d;
using visualization_msgs::msg::MarkerArray;

vec2d_list estimate_velocities(vec2d_list old_positions,
  vec2d_list new_positions, double dt, double d_max = 0.5);

// Weighted average of the most recent velocity estimates, per agent.
// prev_velocities: one vec2d_list per past frame, oldest first, newest last.
// weights: one weight per frame, aligned so the last weight applies to the newest frame.
// Agents are indexed by the newest frame; older frames with fewer agents only contribute where defined.
vec2d_list smooth_velocities(const std::vector<vec2d_list> & prev_velocities,
  const std::vector<double> & weights);

MarkerArray create_velocity_markers(vec2d_list positions,
  vec2d_list velocities, rclcpp::Time time);
