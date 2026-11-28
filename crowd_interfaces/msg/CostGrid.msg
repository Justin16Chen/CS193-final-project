# Planner cost grid for one planning cycle: one cell per (speed, angular velocity)
# candidate. 2D arrays are flattened row-major with speed as the row:
#   value[speed_idx * ang_vels.size() + ang_vel_idx]
# Each component is already multiplied by its weight from controller_params.yaml,
# and total is the sum of the four components.
std_msgs/Header header

float64[] speeds              # m/s, length S
float64[] ang_vels            # rad/s, length A

float64[] total               # S*A
float64[] dist                # S*A
float64[] away_from_goal      # S*A
float64[] diff_from_prev      # S*A
float64[] diff_from_preferred # S*A

int32 best_speed_idx
int32 best_ang_vel_idx

# Robot state the grid was computed from, for labelling plots.
float64 robot_x
float64 robot_y
float64 robot_yaw
float64 goal_x
float64 goal_y
uint32 num_agents

# predict_params the grid was computed with (values from controller_params.yaml
# or whatever was last pushed with `ros2 param load`).
float64 ahead_time
float64 ignore_dist_threshold
float64 dist_cost_weight
float64 away_from_goal_weight
float64 diff_from_prev_weight
float64 diff_from_preferred_weight
