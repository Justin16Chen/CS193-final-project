#include "rclcpp/rclcpp.hpp"
#include <cmath>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "crowd_sim/rviz_helper.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#define PI (3.14159265)
#define DEG2RAD(x) (PI * (x) / 180.)
#define DIST_TO_STOP (1.5)
#define INITIAL_FWD_SPEED (0.0)
#define LIDAR_SCAN_HEIGHT (0.4)

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
using Eigen::Vector3d;
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

    odo_sub_ = create_subscription<Odometry>("model/robot/odometry", 10, [this] (const Odometry& odo) {
      odo_pose_ = odo.pose.pose;
    });

    scan_sub_ = create_subscription<LaserScan>("lidar", 10,
      [this] (const LaserScan & scan) {

        MarkerArray marker_array;
        Pose robot_pose = get_robot_pose();
        Vector3d robot_position = Vector3d(robot_pose.position.x, robot_pose.position.y, LIDAR_SCAN_HEIGHT);
        double yaw = tf2::getYaw(robot_pose.orientation);
        double start_angle = yaw + scan.angle_min;

        Marker clear;
        clear.header.frame_id = "odom";
        clear.header.stamp = now();
        clear.ns = "arrows";
        clear.action = Marker::DELETEALL;
        marker_array.markers.push_back(clear);
        for (long unsigned int i = 0; i < scan.ranges.size(); i++) {
          double angle = start_angle + i * scan.angle_increment;
          Vector3d dir(std::cos(angle), std::sin(angle), 0.);
          double dist = scan.ranges[i];
          if (!std::isfinite(dist) || dist > scan.range_max) {
            dist = scan.range_max;
          }
          else if (dist < scan.range_min) {
            dist = scan.range_min;
          }
          Vector3d offset = dir * dist;
          Vector3d end = robot_position + offset;
          Marker marker = create_marker_arrow(i, now(), robot_position, end, 0.2, 0.8, 0.2);
          marker_array.markers.push_back(marker);
        }
        lidar_pub_->publish(marker_array);
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
