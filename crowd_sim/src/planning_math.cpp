#include "crowd_sim/planning_math.hpp"
#include <cmath>


double calculate_angle_traveled(OutputInfo output_info) {
  if (std::abs(output_info.ang_vel) < 0.00001)
    return 0;
  double radius_of_curvature = output_info.speed / output_info.ang_vel;
  double arc_length_traveled = output_info.speed * output_info.time;
  return arc_length_traveled / radius_of_curvature;
}

vec2d_list get_future_positions(const vec2d_list& positions, const vec2d_list& velocities, double time) {
  vec2d_list future_positions(positions.size(), Vector2d::Zero());
  for (unsigned long int i = 0; i < positions.size(); i++) {
    future_positions[i] = positions[i] + velocities[i] * time;
  }
  return future_positions;
}

Vector2d get_future_robot_position(Vector2d robot_position, double robot_heading, OutputInfo output_info) {
  Vector2d local_displacement;
  if (std::abs(output_info.ang_vel) < 0.00001) { // practically a straight line
    local_displacement = Vector2d(output_info.speed * output_info.time, 0);
  }
  else {
    double radius_of_curvature = output_info.speed / output_info.ang_vel;
    double angle_traveled = calculate_angle_traveled(output_info);
    if (angle_traveled >= 0) 
      local_displacement = (Vector2d(cos(angle_traveled), sin(angle_traveled)) - Vector2d(1, 0)) * radius_of_curvature;
    else {
      angle_traveled += M_PI;
      local_displacement = (Vector2d(cos(angle_traveled), sin(angle_traveled)) - Vector2d(-1, 0)) * radius_of_curvature;
    }
  }
  return Eigen::Rotation2Dd(robot_heading) * local_displacement + robot_position;
}

double calculate_dist_cost(PredictParams predict_params, const vec2d_list& positions, Vector2d robot_position) {
  if (positions.size() == 0) {
    return 0;
  }
  double total = 0;
  for (Vector2d position : positions) {
    total += (position - robot_position).squaredNorm();
  }
  return predict_params.dist_cost / total;
}

double calculate_away_from_goal_cost(PredictParams predict_params, Vector2d robot_position, double robot_heading,
  Vector2d goal_position, OutputInfo output_info) {

  Vector2d robot_to_goal_dir = (goal_position - robot_position).normalized();
  double angle_traveled = calculate_angle_traveled(output_info);
  double final_angle = angle_traveled + robot_heading;
  Vector2d velocity_dir = Vector2d(cos(final_angle), sin(final_angle));
  double dot_prod = robot_to_goal_dir.dot(velocity_dir);
  double corrected = -(dot_prod * 0.5 - 0.5); // 0 is good, 1 is bad
  corrected *= corrected;
  return corrected * output_info.speed * predict_params.away_from_goal_cost; // scale accordingly
}

double calculate_diff_from_prev_output_cost(PredictParams predict_params, OutputInfo cur_output, OutputInfo prev_output) {
  double prev_angle_traveled = calculate_angle_traveled(prev_output);
  double cur_angle_traveled = calculate_angle_traveled(cur_output);
  Vector2d prev_vel = Vector2d(cos(prev_angle_traveled), sin(prev_angle_traveled)) * prev_output.speed;
  Vector2d cur_vel = Vector2d(cos(cur_angle_traveled), sin(cur_angle_traveled)) * cur_output.speed;
  return (cur_vel - prev_vel).squaredNorm() * predict_params.diff_from_prev_output_cost;
}

double calculate_diff_from_preferred_speed_cost(PredictParams predict_params,
  double cur_speed) {
    double diff = std::abs(cur_speed - predict_params.preferred_speed);
    return diff * diff * predict_params.diff_from_preferred_speed_cost;
}
