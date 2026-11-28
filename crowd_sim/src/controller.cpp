#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "crowd_sim/lidar.hpp"
#include "crowd_sim/path_planning.hpp"
#include "crowd_sim/utils.hpp"
#include "crowd_sim/rviz_helper.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using rclcpp::Node;
using rclcpp::Subscription;
using rclcpp::Publisher;
using sensor_msgs::msg::LaserScan;
using sensor_msgs::msg::Imu;
using geometry_msgs::msg::Twist;
using std::shared_ptr;
using visualization_msgs::msg::MarkerArray;
using visualization_msgs::msg::Marker;
using nav_msgs::msg::Odometry;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::Point;
using std_srvs::srv::Trigger;
using Eigen::Vector3d;


class Controller : public Node {
public:
  Controller() : Node("controller") {
    initial_pose_ = get_initial_pose_();
    goal_position_=  get_goal_position_();
    goal_dist_threshold_squared_ = get_goal_dist_threshold_squared_();

    EstimationParams est_params = get_estimation_params_();
    SmoothParams smooth_params = get_smooth_params_();
    declare_predict_params_();
    predict_params_ = read_predict_params_();

    // predict_params.* are live: `ros2 param load /controller controller_params.yaml`
    // (or plot_costs.py --reload) pushes the yaml to the running node and this
    // callback rebuilds the struct, so the next scan and the next replan use it.
    param_cb_handle_ = add_post_set_parameters_callback(
      [this] (const std::vector<rclcpp::Parameter>& params) {
        for (const auto& p : params) {
          if (p.get_name().rfind("predict_params.", 0) == 0) {
            predict_params_ = read_predict_params_();
            RCLCPP_INFO(get_logger(), "predict_params updated (%s)", p.get_name().c_str());
            return;
          }
        }
      });

    // Re-run the planner on the inputs from the most recent scan and republish
    // the cost grid, without publishing cmd_vel. Works while Gazebo is paused,
    // which is what plot_costs.py --reload relies on.
    replan_srv_ = create_service<Trigger>("~/replan",
      [this] (const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr res) {
        if (!have_plan_inputs_) {
          res->success = false;
          res->message = "no lidar scan processed yet, nothing to replan from";
          return;
        }
        PlanResult plan = calculate_output(last_agent_pos_, last_agent_vel_, predict_params_,
          last_lidar_pose_, goal_position_, prev_output_);
        publish_cost_grid_(plan.grid);
        res->success = true;
        res->message = "speed " + std::to_string(plan.cmd.linear.x) +
          " m/s, ang vel " + std::to_string(RAD2DEG(plan.cmd.angular.z)) + " deg/s";
      });

    lidar_pos_pub_ = create_publisher<Marker>("lidar_position", 10);
    robot_heading_pub_ = create_publisher<Marker>("robot_heading", 10);
    goal_pos_pub_= create_publisher<Marker>("goal_position", 10);

    odo_sub_ = create_subscription<Odometry>("model/robot/odometry", 10, [this] (const Odometry& odo) {
      odo_pose_ = odo.pose.pose;
      Pose lidar_pose = get_lidar_pose();
      Vector3d lidar_position = Vector3d(lidar_pose.position.x, lidar_pose.position.y, 0.);
      Vector3d lidar_to_goal = lidar_position - Vector3d(goal_position_.x(), goal_position_.y(), 0);
      if (lidar_to_goal.squaredNorm() < goal_dist_threshold_squared_) {
        goal_position_ = reassign_goal_position_(goal_position_);
      }
      double yaw = tf2::getYaw(odo_pose_.orientation);
      lidar_pos_pub_->publish(create_marker_cylinder("lidar_position", 0, now(),
        lidar_position, 0.1, 1.5,
        make_color(0.2, 0.9, 0.2, 1.)));
      robot_heading_pub_->publish(create_marker_arrow("robot_heading", 0, now(),
        lidar_position, lidar_position + Vector3d(cos(yaw), sin(yaw), 0),
        make_color(0.2, 0.9, 0.2, 1.)));
      goal_pos_pub_->publish(create_marker_cylinder("goal_position", 0, now(),
        Vector3d(goal_position_.x(), goal_position_.y(), 0.), 0.1, 1.5,
        make_color(0.2, 0.2, 0.9, 1.)));
    });

    ground_truth_position_pub_ = create_publisher<Marker>("ground_truth_lidar_position", 10);
    ground_truth_heading_pub_ = create_publisher<Marker>("ground_truth_robot_heading", 10);
    ground_truth_sub_ = create_subscription<Odometry>("model/robot/ground_truth", 10, [this] (const Odometry &odo) {
      ground_truth_odo_pose_ = odo.pose.pose;
      Pose gt_lidar_pose = get_ground_truth_lidar_pose();
      Vector3d gt_lidar_position = Vector3d(gt_lidar_pose.position.x, gt_lidar_pose.position.y, 0);
      double yaw = tf2::getYaw(gt_lidar_pose.orientation);

      ground_truth_position_pub_->publish(create_marker_cylinder("ground_truth_lidar_position", 0, now(),
        gt_lidar_position, 0.1, 1.5,
        make_color(0.2, 0.5, 0.9, 0.9)));
      ground_truth_heading_pub_->publish(create_marker_arrow("ground_truth_robot_heading", 0, now(),
        gt_lidar_position, gt_lidar_position + Vector3d(cos(yaw), sin(yaw), 0),
        make_color(0.2, 0.5, 0.9, 1.)));
    });

    est_agent_pos_pub_ = create_publisher<MarkerArray>("est_agent_positions", 10);
    est_agent_vel_pub_ = create_publisher<MarkerArray>("est_agent_velocities", 10);
    lidar_pub_ = create_publisher<MarkerArray>("lidar_distances", 10);
    cmd_vel_pub_ = create_publisher<Twist>("cmd_vel", 10);
    // Transient-local, depth 1: the latest grid stays cached, so plot_costs.py
    // can be started after the sim is paused and still receive the grid for
    // the paused instant.
    cost_grid_pub_ = create_publisher<CostGrid>("cost_grid", rclcpp::QoS(1).transient_local());

    last_update_time_ = now();
    scan_sub_ = create_subscription<LaserScan>("lidar", 10,
      [this, est_params, smooth_params] (const LaserScan & raw_scan) {
        rclcpp::Time t = now();
        double dt = std::clamp((t - last_update_time_).seconds(), 0.0, kMaxDt);
        last_update_time_ = t;

        Pose lidar_pose = get_lidar_pose();
        lidar_pub_->publish(create_lidar_markers(raw_scan, now(), lidar_pose));

        vec2d_list est_agent_pos = estimate_agent_positions(raw_scan, est_params, lidar_pose);
        est_agent_pos_pub_->publish(create_est_position_markers(est_agent_pos, now()));

        vec2d_list est_agent_vel = estimate_velocities(prev_agent_pos_, est_agent_pos, dt, est_params.max_position_dist_diff);
        prev_agent_vels_.push_back(est_agent_vel);
        vec2d_list smoothed_vel = smooth_velocities(prev_agent_vels_, smooth_params);
        est_agent_vel_pub_->publish(create_velocity_markers(est_agent_pos, smoothed_vel, now()));

        prev_agent_pos_ = est_agent_pos;
        if (prev_agent_vels_.size() > smooth_params.weights.size()) {
          prev_agent_vels_.erase(prev_agent_vels_.begin());
        }

        ///*
        last_agent_pos_ = est_agent_pos;
        last_agent_vel_ = est_agent_vel;
        last_lidar_pose_ = lidar_pose;
        have_plan_inputs_ = true;
        PlanResult plan = calculate_output(est_agent_pos, est_agent_vel, predict_params_, lidar_pose, goal_position_, prev_output_);
        Twist commanded_output = plan.cmd;
        publish_cost_grid_(plan.grid);
        cmd_vel_pub_->publish(commanded_output);
        //Twist test_output;
        //test_output.linear.set__x(1);
        //test_output.angular.set__z(DEG2RAD(20));
        //cmd_vel_pub_->publish(test_output);
        cmd_vel_pub_->publish(commanded_output);
        prev_output_ = commanded_output;
        //*/
      }
    );
  }

  Pose get_odo_pose() {
    tf2::Transform t_init, t_odo;
    tf2::fromMsg(initial_pose_, t_init);
    tf2::fromMsg(odo_pose_, t_odo);
    Pose out;
    tf2::toMsg(t_init * t_odo, out);
    return out;
  }
  Pose get_ground_truth_odo_pose() {
    return ground_truth_odo_pose_;
  }

  // Lidar frame in the world: robot (odometry origin, between the drive wheels)
  // composed with the fixed chassis -> lidar offset from the SDF.
  Pose get_lidar_pose() {
    tf2::Transform t_robot, t_lidar;
    tf2::fromMsg(get_odo_pose(), t_robot);
    t_lidar.setIdentity();
    t_lidar.setOrigin(tf2::Vector3(kLidarOffsetX, kLidarOffsetY, kLidarOffsetZ));
    Pose out;
    tf2::toMsg(t_robot * t_lidar, out);
    return out;
  }
  Pose get_ground_truth_lidar_pose() {
    tf2::Transform t_robot, t_lidar;
    tf2::fromMsg(get_ground_truth_odo_pose(), t_robot);
    t_lidar.setIdentity();
    t_lidar.setOrigin(tf2::Vector3(kLidarOffsetX, kLidarOffsetY, kLidarOffsetZ));
    Pose out;
    tf2::toMsg(t_robot * t_lidar, out);
    return out;
  }
private:
  static constexpr double kMaxDt = 0.1; // seconds
  // lidar_frame relative to the model origin (chassis at x=0.1667, lidar 0.3333 above chassis)
  static constexpr double kLidarOffsetX = 0.1667;
  static constexpr double kLidarOffsetY = 0.0;
  static constexpr double kLidarOffsetZ = 0.1333 + 0.3333;
  shared_ptr<Subscription<LaserScan>> scan_sub_;
  shared_ptr<Subscription<Odometry>> odo_sub_;
  shared_ptr<Subscription<Odometry>> ground_truth_sub_;
  shared_ptr<Publisher<Marker>> ground_truth_position_pub_;
  shared_ptr<Publisher<Marker>> ground_truth_heading_pub_;
  shared_ptr<Publisher<Twist>> cmd_vel_pub_;
  shared_ptr<Publisher<CostGrid>> cost_grid_pub_;
  shared_ptr<rclcpp::Service<Trigger>> replan_srv_;
  rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr param_cb_handle_;
  shared_ptr<Publisher<MarkerArray>> lidar_pub_;
  shared_ptr<Publisher<MarkerArray>> est_agent_pos_pub_;
  shared_ptr<Publisher<MarkerArray>> est_agent_vel_pub_;
  shared_ptr<Publisher<Marker>> lidar_pos_pub_;
  shared_ptr<Publisher<Marker>> robot_heading_pub_;
  shared_ptr<Publisher<Marker>> goal_pos_pub_;

  rclcpp::Time last_update_time_;
  Pose initial_pose_;
  Vector2d goal_position_;
  double goal_dist_threshold_squared_;
  Pose odo_pose_;
  Pose ground_truth_odo_pose_;
  bool prev_less_than_range = false;
  vec2d_list prev_agent_pos_;
  std::vector<vec2d_list> prev_agent_vels_;
  Twist prev_output_;

  // live planner params + inputs of the last scan, for the replan service
  PredictParams predict_params_;
  vec2d_list last_agent_pos_;
  vec2d_list last_agent_vel_;
  Pose last_lidar_pose_;
  bool have_plan_inputs_ = false;

  void publish_cost_grid_(CostGrid grid) {
    grid.header.stamp = now();
    grid.header.frame_id = "world";
    cost_grid_pub_->publish(grid);
  }

  Pose get_initial_pose_() {
    declare_parameter("start_x", 0.);
    declare_parameter("start_y", 0.);
    declare_parameter("start_yaw", 0.);
    double start_x = get_parameter("start_x").as_double();
    double start_y = get_parameter("start_y").as_double();
    double start_yaw = get_parameter("start_yaw").as_double();
    Pose initial_pose;
    initial_pose.position.x = start_x;
    initial_pose.position.y = start_y;
    initial_pose.position.z = 0;
    initial_pose.orientation.x = 0;
    initial_pose.orientation.y = 0;
    initial_pose.orientation.z = std::sin(start_yaw * 0.5);
    initial_pose.orientation.w = std::cos(start_yaw * 0.5);
    return initial_pose;
  }
  Vector2d get_goal_position_() {
    declare_parameter("goal_x", 0.);
    declare_parameter("goal_y", 0.);
    return Vector2d(
      get_parameter("goal_x").as_double(),
      get_parameter("goal_y").as_double()
    );
  }
  double get_goal_dist_threshold_squared_() {
    declare_parameter("goal_dist_threshold", 0.5);
    double dist = get_parameter("goal_dist_threshold").as_double();
    return dist * dist;
  }
  EstimationParams get_estimation_params_() {
    declare_parameter("est_params.max_dist_diff", 0.2);
    declare_parameter("est_params.agent_width", 0.4);
    declare_parameter("est_params.max_position_dist_diff", 0.2);
    double max_dist_diff = get_parameter("est_params.max_dist_diff").as_double();
    double agent_width = get_parameter("est_params.agent_width").as_double();
    double max_position_dist_diff = get_parameter("est_params.max_position_dist_diff").as_double();
    return EstimationParams{
      .max_lidar_dist_diff = max_dist_diff,
      .agent_width = agent_width,
      .max_position_dist_diff = max_position_dist_diff
    };
  }
  SmoothParams get_smooth_params_() {
    declare_parameter("smooth_params.weights", std::vector<double>{1.0, 1.0, 1.0, 1.0, 1.0});
    std::vector<double> vel_weights = get_parameter("smooth_params.weights").as_double_array();
    return SmoothParams{
      .weights = vel_weights
    };
  }
  Vector2d reassign_goal_position_(Vector2d old_goal) {
    return Vector2d(-old_goal.x(), old_goal.y());
  }
  void declare_predict_params_() {
    declare_parameter("predict_params.ahead_time", 0.5);
    declare_parameter("predict_params.time_resolution", 0.5);
    declare_parameter("predict_params.min_speed", 0.5);
    declare_parameter("predict_params.max_speed", 0.5);
    declare_parameter("predict_params.preferred_speed", 0.5);
    declare_parameter("predict_params.speed_resolution", 0.5);
    declare_parameter("predict_params.max_angular_vel_deg", 0.5);
    declare_parameter("predict_params.angular_vel_resolution_deg", 0.5);
    declare_parameter("predict_params.ignore_dist_threshold", 2.);
    declare_parameter("predict_params.dist_cost", 1.);
    declare_parameter("predict_params.away_from_goal_cost", 0.5);
    declare_parameter("predict_params.diff_from_prev_output_cost", 0.5);
    declare_parameter("predict_params.diff_from_preferred_speed_cost", 0.5);
  }
  // Reads the current parameter values; safe to call after any parameter update.
  PredictParams read_predict_params_() {
    double ignore_dist_threshold = get_parameter("predict_params.ignore_dist_threshold").as_double();
    return PredictParams{
      .ahead_time = get_parameter("predict_params.ahead_time").as_double(),
      .time_resolution = get_parameter("predict_params.time_resolution").as_double(),
      .min_speed = get_parameter("predict_params.min_speed").as_double(),
      .max_speed = get_parameter("predict_params.max_speed").as_double(),
      .preferred_speed = get_parameter("predict_params.preferred_speed").as_double(),
      .speed_resolution = get_parameter("predict_params.speed_resolution").as_double(),
      .max_angular_vel = DEG2RAD(get_parameter("predict_params.max_angular_vel_deg").as_double()),
      .angular_vel_resolution = DEG2RAD(get_parameter("predict_params.angular_vel_resolution_deg").as_double()),
      .ignore_dist_threshold_squared = ignore_dist_threshold * ignore_dist_threshold,
      .dist_cost = get_parameter("predict_params.dist_cost").as_double(),
      .away_from_goal_cost = get_parameter("predict_params.away_from_goal_cost").as_double(),
      .diff_from_prev_output_cost = get_parameter("predict_params.diff_from_prev_output_cost").as_double(),
      .diff_from_preferred_speed_cost = get_parameter("predict_params.diff_from_preferred_speed_cost").as_double()
    };
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Controller>());
  rclcpp::shutdown();
}
