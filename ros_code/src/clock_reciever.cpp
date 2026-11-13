#include "rclcpp/rclcpp.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include <memory.h>

class ClockReciever : public rclcpp::Node {
public:
  ClockReciever() : Node("clock_reciever") {
    clock_sub_ = create_subscription<rosgraph_msgs::msg::Clock>("/clock", 10,
        [this] (const rosgraph_msgs::msg::Clock & clock) {
        int seconds = clock.clock.sec;
        //RCLCPP_INFO(get_logger(), "clock seconds: %d\n", seconds);
      });
  }
private:
  std::shared_ptr<rclcpp::Subscription<rosgraph_msgs::msg::Clock>> clock_sub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ClockReciever>());
  rclcpp::shutdown();
}
