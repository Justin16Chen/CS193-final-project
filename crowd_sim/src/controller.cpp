#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "crowd_sim/lidar.hpp"
#include "crowd_sim/path_planning.hpp"
#include "crowd_sim/utils.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#define PI (3.14159265)
#define DEG2RAD(x) (PI * (x) / 180.)
#define DIST_TO_STOP (1.5)
#define INITIAL_FWD_SPEED (0.0)

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


class Controller : public Node {
public:
  Controller() : Node("controller") {
    cmd_vel_.linear.x = INITIAL_FWD_SPEED;

    declare_parameter("start_x", 0.);
    declare_parameter("start_y", 0.);
    declare_parameter("start_yaw", 0.);
    double start_x = get_parameter("start_x").as_double();
    double start_y = get_parameter("start_y").as_double();
    double start_yaw = get_parameter("start_yaw").as_double();
    initial_pose_.position.x = start_x;
    initial_pose_.position.y = start_y;
    initial_pose_.position.z = 0;
    initial_pose_.orientation.x = 0;
    initial_pose_.orientation.y = 0;
    initial_pose_.orientation.z = std::sin(start_yaw * 0.5);
    initial_pose_.orientation.w = std::cos(start_yaw * 0.5);

    declare_parameter("est_params.max_dist_diff", 0.2);
    declare_parameter("est_params.agent_width", 0.4);
    double max_dist_diff = get_parameter("est_params.max_dist_diff").as_double();
    double agent_width = get_parameter("est_params.agent_width").as_double();
    EstimationParams est_params = {
      .max_dist_diff = max_dist_diff,
      .agent_width = agent_width
    };

    declare_parameter("smooth_params.num_prev_vels", 5);
    declare_parameter("smooth_params.weights", std::vector<double>{1.0, 1.0, 1.0, 1.0, 1.0});
    num_prev_vels_ = get_parameter("smooth_params.num_prev_vels").as_int();
    vel_weights_ = get_parameter("smooth_params.weights").as_double_array();
    if (num_prev_vels_ < 1) {
      RCLCPP_WARN(get_logger(), "smooth_params.num_prev_vels must be >= 1, using 1");
      num_prev_vels_ = 1;
    }
    if (vel_weights_.size() != static_cast<size_t>(num_prev_vels_)) {
      RCLCPP_WARN(get_logger(),
        "smooth_params.weights has %zu entries but num_prev_vels is %d; using uniform weights",
        vel_weights_.size(), num_prev_vels_);
      vel_weights_.assign(num_prev_vels_, 1.0);
    }

    odo_sub_ = create_subscription<Odometry>("model/robot/odometry", 10, [this] (const Odometry& odo) {
      odo_pose_ = odo.pose.pose;
    });

    est_agent_pos_pub_ = create_publisher<MarkerArray>("est_agent_positions", 10);
    est_agent_vel_pub_ = create_publisher<MarkerArray>("est_agent_velocities", 10);

    last_update_time_ = now();
    scan_sub_ = create_subscription<LaserScan>("lidar", 10,
      [this, est_params] (const LaserScan & raw_scan) {
        rclcpp::Time t = now();
        double dt = std::clamp((t - last_update_time_).seconds(), 0.0, kMaxDt);
        last_update_time_ = t;

        Pose lidar_pose = get_lidar_pose();
        lidar_pub_->publish(create_lidar_markers(raw_scan, now(), lidar_pose));

        vec2d_list est_agent_pos = estimate_agent_positions(raw_scan, est_params, lidar_pose);
        est_agent_pos_pub_->publish(create_est_position_markers(est_agent_pos, now()));

        vec2d_list est_agent_vel = estimate_velocities(prev_agent_pos_, est_agent_pos, dt);
        prev_agent_vels_.push_back(est_agent_vel);
        vec2d_list smoothed_vel = smooth_velocities(prev_agent_vels_, vel_weights_);
        est_agent_vel_pub_->publish(create_velocity_markers(est_agent_pos, smoothed_vel, now()));

        prev_agent_pos_ = est_agent_pos;
        if (prev_agent_vels_.size() > static_cast<size_t>(num_prev_vels_)) {
          prev_agent_vels_.erase(prev_agent_vels_.begin());
        }
      }
    );

    cmd_vel_pub_ = create_publisher<Twist>("cmd_vel", 10);
    lidar_pub_ = create_publisher<MarkerArray>("lidar_distances", 10);

    declare_parameter("controller_frequency", 20.0);
    double period_sec = 1. / get_parameter("controller_frequency").as_double();
    auto period_nano = rclcpp::duration_cast<rclcpp::nanoseconds>(rclcpp::duration<double>(period_sec));
    cmd_vel_timer_ = create_wall_timer(period_nano,
      [this] () {
        cmd_vel_pub_->publish(cmd_vel_);
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
  shared_ptr<rclcpp::TimerBase> cmd_vel_timer_;

  rclcpp::Time last_update_time_;
  Pose initial_pose_;
  Pose odo_pose_;
  Twist cmd_vel_;
  bool prev_less_than_range = false;
  vec2d_list prev_agent_pos_;
  std::vector<vec2d_list> prev_agent_vels_;
  int num_prev_vels_ = 5;
  std::vector<double> vel_weights_;

  void stop() {
    cmd_vel_.linear.x = 0;
    cmd_vel_.linear.y = 0;
    cmd_vel_.linear.z = 0;
    cmd_vel_.angular.x = 0;
    cmd_vel_.angular.y = 0;
    cmd_vel_.angular.z = 0;
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Controller>());
  rclcpp::shutdown();
}
