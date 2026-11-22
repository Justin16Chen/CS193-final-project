#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "crowd_sim/lidar.hpp"
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

    odo_sub_ = create_subscription<Odometry>("model/robot/odometry", 10, [this] (const Odometry& odo) {
      odo_pose_ = odo.pose.pose;
    });

    est_agent_pos_pub_ = create_publisher<MarkerArray>("est_agent_positions", 10);

    scan_sub_ = create_subscription<LaserScan>("lidar", 10,
      [this, est_params] (const LaserScan & raw_scan) {
        Pose robot_pose = get_robot_pose();
        lidar_pub_->publish(create_lidar_markers(raw_scan, now(), robot_pose));

        std::vector<Vector2d> est_agent_pos = estimate_agent_positions(raw_scan, est_params, robot_pose);
        est_agent_pos_pub_->publish(create_est_position_markers(est_agent_pos, now()));
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
private:
  shared_ptr<Subscription<LaserScan>> scan_sub_;
  shared_ptr<Subscription<Odometry>> odo_sub_;
  shared_ptr<Publisher<Twist>> cmd_vel_pub_;
  shared_ptr<Publisher<MarkerArray>> lidar_pub_;
  shared_ptr<Publisher<MarkerArray>> est_agent_pos_pub_;
  shared_ptr<rclcpp::TimerBase> cmd_vel_timer_;

  Pose initial_pose_;
  Pose odo_pose_;
  Twist cmd_vel_;
  bool prev_less_than_range = false;

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
