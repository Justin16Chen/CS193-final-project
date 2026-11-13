#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <memory.h>

#define PI (3.14159265)
#define DEG2RAD(x) (PI * (x) / 180.)
#define MIN_ANG_DEG (-5)
#define MAX_ANG_DEG (5)
#define DIST_TO_STOP (1.5)
#define INITIAL_FWD_SPEED (0.5)

using namespace rclcpp;
using namespace sensor_msgs::msg;
using namespace geometry_msgs::msg;
using namespace std::chrono;

class Controller : public Node {
public:
  Controller() : Node("controller") {
    cmd_vel_.linear.x = INITIAL_FWD_SPEED;

    declare_parameter("print_scans", false);
    bool print_scans = get_parameter("print_scans").as_bool();

    scan_sub_ = create_subscription<LaserScan>("lidar", 10,
      [this, print_scans] (const LaserScan & scan) {

        bool less_than_range = false;
        for (int i = 0; i < scan.ranges.size(); i++) {
          double range = scan.ranges[i];
          if (range < DIST_TO_STOP) {
            less_than_range = true;
            this->stop();
            break;
          }
        }
        if (print_scans && (less_than_range != prev_less_than_range)) {
          RCLCPP_INFO(get_logger(), "less than range: %d\n", less_than_range);
          prev_less_than_range = less_than_range;
        }
      }
    );

    cmd_vel_pub_ = create_publisher<Twist>("cmd_vel", 10);

    declare_parameter("controller_frequency", 20.0);
    double period_sec = 1. / get_parameter("controller_frequency").as_double();
    auto period_nano = duration_cast<nanoseconds>(duration<double>(period_sec));
    cmd_vel_timer_ = create_wall_timer(period_nano,
      [this] () {
        this->cmd_vel_pub_->publish(this->cmd_vel_);
      }
    );
  }
private:
  std::shared_ptr<Subscription<LaserScan>> scan_sub_;
  std::shared_ptr<Publisher<Twist>> cmd_vel_pub_;
  std::shared_ptr<rclcpp::TimerBase> cmd_vel_timer_;

  Twist cmd_vel_;
  bool prev_less_than_range = false;

  void stop() {
    cmd_vel_.linear.x = 0;
    cmd_vel_.linear.y = 0;
    cmd_vel_.linear.y = 0;
    cmd_vel_.angular.x = 0;
    cmd_vel_.angular.y = 0;
    cmd_vel_.angular.z = 0;
  }
};

int main(int argc, char **argv) {
  init(argc, argv);
  spin(std::make_shared<Controller>());
  shutdown();
}
