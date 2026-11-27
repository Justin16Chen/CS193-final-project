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
using Eigen::Vector3d;


class Controller : public Node {
public:
  Controller() : Node("controller") {
    initial_pose_ = get_initial_pose_();
    goal_position_=  get_goal_position_();
    EstimationParams est_params = get_estimation_params_();
    SmoothParams smooth_params = get_smooth_params_();
    PredictParams predict_params = get_predict_params_();

    lidar_pos_pub_ = create_publisher<Marker>("lidar_position", 10);
    robot_heading_pub_ = create_publisher<Marker>("robot_heading", 10);
    goal_pos_pub_= create_publisher<Marker>("goal_position", 10);

    odo_sub_ = create_subscription<Odometry>("model/robot/odometry", 10, [this] (const Odometry& odo) {
      odo_pose_ = odo.pose.pose;
      Pose lidar_pose = get_lidar_pose();
      Vector3d lidar_position = Vector3d(lidar_pose.position.x, lidar_pose.position.y, 0.);
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

    est_agent_pos_pub_ = create_publisher<MarkerArray>("est_agent_positions", 10);
    est_agent_vel_pub_ = create_publisher<MarkerArray>("est_agent_velocities", 10);
    lidar_pub_ = create_publisher<MarkerArray>("lidar_distances", 10);
    cmd_vel_pub_ = create_publisher<Twist>("cmd_vel", 10);

    last_update_time_ = now();
    scan_sub_ = create_subscription<LaserScan>("lidar", 10,
      [this, est_params, smooth_params, predict_params] (const LaserScan & raw_scan) {
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
        Twist commanded_output = calculate_output(est_agent_pos, est_agent_vel, predict_params, lidar_pose, goal_position_, prev_output_);
        //cmd_vel_pub_->publish(commanded_output);
        Twist test_output;
        test_output.linear.set__x(1);
        test_output.angular.set__z(DEG2RAD(20));
        cmd_vel_pub_->publish(test_output);
        prev_output_ = commanded_output;
        //*/
      }
    );
  }

  Pose get_robot_pose() {
    tf2::Transform t_init, t_odo;
    tf2::fromMsg(initial_pose_, t_init);
    tf2::fromMsg(odo_pose_, t_odo);
    Pose out;
    tf2::toMsg(t_init * t_odo, out);
    return out;
  }

  // Lidar frame in the world: robot (odometry origin, between the drive wheels)
  // composed with the fixed chassis -> lidar offset from the SDF.
  Pose get_lidar_pose() {
    tf2::Transform t_robot, t_lidar;
    tf2::fromMsg(get_robot_pose(), t_robot);
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
  shared_ptr<Publisher<Twist>> cmd_vel_pub_;
  shared_ptr<Publisher<MarkerArray>> lidar_pub_;
  shared_ptr<Publisher<MarkerArray>> est_agent_pos_pub_;
  shared_ptr<Publisher<MarkerArray>> est_agent_vel_pub_;
  shared_ptr<Publisher<Marker>> lidar_pos_pub_;
  shared_ptr<Publisher<Marker>> robot_heading_pub_;
  shared_ptr<Publisher<Marker>> goal_pos_pub_;

  rclcpp::Time last_update_time_;
  Pose initial_pose_;
  Vector2d goal_position_;
  Pose odo_pose_;
  bool prev_less_than_range = false;
  vec2d_list prev_agent_pos_;
  std::vector<vec2d_list> prev_agent_vels_;
  Twist prev_output_;

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
  PredictParams get_predict_params_() {
    declare_parameter("predict_params.ahead_time", 0.5);
    declare_parameter("predict_params.time_resolution", 0.5);
    declare_parameter("predict_params.min_speed", 0.5);
    declare_parameter("predict_params.max_speed", 0.5);
    declare_parameter("predict_params.preferred_speed", 0.5);
    declare_parameter("predict_params.speed_resolution", 0.5);
    declare_parameter("predict_params.max_angular_vel_deg", 0.5);
    declare_parameter("predict_params.angular_vel_resolution_deg", 0.5);
    declare_parameter("predict_params.dist_cost", 1.);
    declare_parameter("predict_params.away_from_goal_cost", 0.5);
    declare_parameter("predict_params.diff_from_prev_output_cost", 0.5);
    declare_parameter("predict_params.diff_from_preferred_speed_cost", 0.5);
    return PredictParams{
      .ahead_time = get_parameter("predict_params.ahead_time").as_double(),
      .time_resolution = get_parameter("predict_params.time_resolution").as_double(),
      .min_speed = get_parameter("predict_params.min_speed").as_double(),
      .max_speed = get_parameter("predict_params.max_speed").as_double(),
      .preferred_speed = get_parameter("predict_params.preferred_speed").as_double(),
      .speed_resolution = get_parameter("predict_params.speed_resolution").as_double(),
      .max_angular_vel = DEG2RAD(get_parameter("predict_params.max_angular_vel_deg").as_double()),
      .angular_vel_resolution = DEG2RAD(get_parameter("predict_params.angular_vel_resolution_deg").as_double()),
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
