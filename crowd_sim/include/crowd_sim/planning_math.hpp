#pragma once
// Pure math for the path planner. No ROS dependencies so it can be unit tested quickly.

#include <Eigen/Dense>
#include "crowd_sim/utils.hpp"

using Eigen::Vector2d;

// define helpers
#define EPSILON (1e-9)
#define NUM_TIMES(predict) (int)(((predict).ahead_time + EPSILON) / (predict).time_resolution + 1)
#define NUM_SPEEDS(predict) (int)((predict.max_speed - predict.min_speed + EPSILON) / predict.speed_resolution + 1)
#define NUM_ANGULAR_VELS(predict) (int)((2 * predict.max_angular_vel + EPSILON) / predict.angular_vel_resolution + 1)
#define IDX_TO_SPEED(predict, idx) ((predict).min_speed + (idx) * (predict.speed_resolution))
#define IDX_TO_ANGULAR_VEL(predict, idx) (((idx) - (NUM_ANGULAR_VELS(predict) / 2)) * (predict).angular_vel_resolution)
#define IDX_TO_TIME(predict, idx) ((idx) * (predict).time_resolution)


struct PredictParams {
  double ahead_time;
  double time_resolution;
  double min_speed, max_speed, preferred_speed;
  double speed_resolution;
  double max_angular_vel;
  double angular_vel_resolution;

  double ignore_dist_threshold_squared;
  double dist_cost;
  double away_from_goal_cost;
  double diff_from_prev_output_cost;
  double diff_from_preferred_speed_cost;
};

struct OutputInfo {
  double speed;
  double ang_vel;
  double time;
};

typedef std::vector<std::vector<double>> OutputCost;

// Heading change (radians) after driving output_info.speed at output_info.ang_vel for output_info.time.
double calculate_angle_traveled(OutputInfo output_info);

// Positions of each agent time seconds ahead, assuming constant velocity.
vec2d_list get_future_positions(const vec2d_list& positions, const vec2d_list& velocities, double time);

// Position of the robot after following a constant-curvature arc described by output_info.
Vector2d get_future_robot_position(Vector2d robot_position, double robot_heading, OutputInfo output_info);

// Sum of squared distances from robot_position to every entry in positions.
double calculate_dist_cost(PredictParams predict_params, const vec2d_list& positions, Vector2d robot_position);

// 0 when the final velocity direction points at the goal, up to speed * away_from_goal_cost when pointing away. quadratic relationship
double calculate_away_from_goal_cost(PredictParams predict_params, Vector2d robot_position,
  double robot_heading, Vector2d goal_position, OutputInfo output_info);

// Squared difference between the velocity vectors of two commands, scaled by diff_from_prev_output_cost.
double calculate_diff_from_prev_output_cost(PredictParams predict_params,
  OutputInfo cur_output, OutputInfo prev_output);

double calculate_diff_from_preferred_speed_cost(PredictParams predict_params,
  double cur_speed);
