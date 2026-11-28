#include "crowd_sim/path_planning.hpp"
#include "crowd_sim/match.hpp"
#include "crowd_sim/rviz_helper.hpp"
#include "crowd_sim/utils.hpp"
#include <assert.h>
#include <algorithm>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using Eigen::Vector3d;

std::vector<Vector2d> estimate_velocities(std::vector<Vector2d> old_positions,
                                          std::vector<Vector2d> new_positions,
                                          double dt, double d_max) {
    // one velocity per NEW position; unmatched (newly seen) agents default to zero
    std::vector<Vector2d> vel(new_positions.size(), Vector2d::Zero());

    MatchResult r = match(old_positions, new_positions, d_max);
    for (auto [i, j] : r.matches)
        vel[j] = (new_positions[j] - old_positions[i]) / dt;

    return vel;
}

vec2d_list smooth_velocities(const std::vector<vec2d_list> & prev_velocities,
                             const SmoothParams& smooth_params) {
    if (prev_velocities.empty()) return {};

    const vec2d_list & newest = prev_velocities.back();
    vec2d_list out(newest.size(), Vector2d::Zero());
    std::vector<double> weight_sum(newest.size(), 0.0);

    // Align the last weight with the newest frame; if there are more frames than
    // weights, the oldest frames are ignored.
    size_t n = std::min(prev_velocities.size(), smooth_params.weights.size());
    size_t frame0 = prev_velocities.size() - n;
    size_t weight0 = smooth_params.weights.size() - n;

    for (size_t k = 0; k < n; k++) {
        const vec2d_list & frame = prev_velocities[frame0 + k];
        double w = smooth_params.weights[weight0 + k];
        for (size_t i = 0; i < out.size() && i < frame.size(); i++) {
            out[i] += w * frame[i];
            weight_sum[i] += w;
        }
    }

    for (size_t i = 0; i < out.size(); i++) {
        if (weight_sum[i] > 0.0) out[i] /= weight_sum[i];
    }
    return out;
}


MarkerArray create_velocity_markers(vec2d_list positions, 
  vec2d_list velocities, rclcpp::Time time) {
  assert(positions.size() == velocities.size());
  MarkerArray markers;
  markers.markers.push_back(create_clear_marker("velocities", time));

  ColorRGBA color = make_color(0.2, 0.2, 0.9, 1.);
  for (unsigned long int i = 0; i < positions.size(); i++) {
    Vector3d pos1(positions[i].x(), positions[i].y(), 0);
    Vector2d end = positions[i] + velocities[i];
    Vector3d pos2(end.x(), end.y(), 0);
    markers.markers.push_back(create_marker_arrow("velocities", i + 1, time, pos1, pos2, color));
  }

  return markers;
}


PlanResult calculate_output(const vec2d_list& positions, const vec2d_list& velocities,
  PredictParams predict, Pose robot_pose, Vector2d goal_position, Twist prev_output) {

  Vector2d robot_position(robot_pose.position.x, robot_pose.position.y);
  double robot_heading = tf2::getYaw(robot_pose.orientation);
  OutputInfo prev_output_info = {
    .speed = prev_output.linear.x,
    .ang_vel = prev_output.angular.z,
    .time = 0 // updated in the for loop
  };
  int num_times = NUM_TIMES(predict);
  int num_speeds = NUM_SPEEDS(predict);
  int num_angular_vels = NUM_ANGULAR_VELS(predict);
  // needs to be odd for symmetry and to guarantee angular vel of 0 is included
  if (num_angular_vels % 2 == 0)
    num_angular_vels++;

  // Row-major (speed, ang_vel) grids; total is what the argmin runs over.
  PlanResult result;
  CostGrid& grid = result.grid;
  size_t n_cells = (size_t)num_speeds * num_angular_vels;
  grid.total.assign(n_cells, 0.);
  grid.dist.assign(n_cells, 0.);
  grid.away_from_goal.assign(n_cells, 0.);
  grid.diff_from_prev.assign(n_cells, 0.);
  grid.diff_from_preferred.assign(n_cells, 0.);
  grid.speeds.reserve(num_speeds);
  for (int i = 0; i < num_speeds; i++) grid.speeds.push_back(IDX_TO_SPEED(predict, i));
  grid.ang_vels.reserve(num_angular_vels);
  for (int i = 0; i < num_angular_vels; i++) grid.ang_vels.push_back(IDX_TO_ANGULAR_VEL(predict, i));

  int min_speed_idx = 0;
  int min_ang_vel_idx = 0;
  for (int speed_idx = 0; speed_idx < num_speeds; speed_idx++) {
    double speed = IDX_TO_SPEED(predict, speed_idx);
    for (int ang_vel_idx = 0; ang_vel_idx < num_angular_vels; ang_vel_idx++) {
      double ang_vel = IDX_TO_ANGULAR_VEL(predict, ang_vel_idx);
      size_t cell = (size_t)speed_idx * num_angular_vels + ang_vel_idx;
      for (int time_idx = 0; time_idx < num_times; time_idx++) {
        double time = IDX_TO_TIME(predict, time_idx);
        prev_output_info.time = time;
        OutputInfo output_info = {
          .speed = speed,
          .ang_vel = ang_vel,
          .time = time
        };
        vec2d_list future_positions = get_future_positions(positions, velocities, time);
        Vector2d future_robot_position = get_future_robot_position(robot_position, robot_heading, output_info);

        double dist_cost = calculate_dist_cost(predict, positions, future_robot_position);
        double away_from_goal_cost = calculate_away_from_goal_cost(predict, robot_position, robot_heading, goal_position, output_info);
        double diff_from_prev_output_cost = calculate_diff_from_prev_output_cost(predict, output_info, prev_output_info);
        double diff_from_preferred_speed_cost = calculate_diff_from_preferred_speed_cost(predict, speed);

        // total is accumulated per time step in this exact order so it is
        // bit-identical to the pre-CostGrid planner; the per-component arrays
        // are bookkeeping for plot_costs.py only.
        grid.total[cell] += dist_cost + away_from_goal_cost +
          diff_from_prev_output_cost + diff_from_preferred_speed_cost;
        grid.dist[cell] += dist_cost;
        grid.away_from_goal[cell] += away_from_goal_cost;
        grid.diff_from_prev[cell] += diff_from_prev_output_cost;
        grid.diff_from_preferred[cell] += diff_from_preferred_speed_cost;
      }
      size_t min_cell = (size_t)min_speed_idx * num_angular_vels + min_ang_vel_idx;
      if (grid.total[cell] < grid.total[min_cell]) {
        min_speed_idx = speed_idx;
        min_ang_vel_idx = ang_vel_idx;
      }
    }
  }
  grid.best_speed_idx = min_speed_idx;
  grid.best_ang_vel_idx = min_ang_vel_idx;
  grid.robot_x = robot_position.x();
  grid.robot_y = robot_position.y();
  grid.robot_yaw = robot_heading;
  grid.goal_x = goal_position.x();
  grid.goal_y = goal_position.y();
  grid.num_agents = positions.size();
  grid.ahead_time = predict.ahead_time;
  grid.ignore_dist_threshold = std::sqrt(predict.ignore_dist_threshold_squared);
  grid.dist_cost_weight = predict.dist_cost;
  grid.away_from_goal_weight = predict.away_from_goal_cost;
  grid.diff_from_prev_weight = predict.diff_from_prev_output_cost;
  grid.diff_from_preferred_weight = predict.diff_from_preferred_speed_cost;

  double best_speed = IDX_TO_SPEED(predict, min_speed_idx);
  double best_ang_vel = IDX_TO_ANGULAR_VEL(predict, min_ang_vel_idx);
  //RCLCPP_INFO(rclcpp::get_logger("calculating cmd_vel"), "best speed: (%.5lf) | best ang_vel: (%.5lf)", best_speed, RAD2DEG(best_ang_vel));
  result.cmd.linear.set__x(best_speed);
  result.cmd.angular.set__z(best_ang_vel);
  return result;
}
