#include "rclcpp/rclcpp.hpp"
#include "crowd_interfaces/msg/person_array.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "agent.hpp"

#include <stdlib.h>
#include <time.h>

using rclcpp::Node;
using rclcpp::duration;
using rclcpp::TimerBase;
using rclcpp::Publisher;
using std::shared_ptr;
using std::make_shared;
using crowd_interfaces::msg::PersonArray;
using crowd_interfaces::msg::Person;
using visualization_msgs::msg::MarkerArray;

class Crowd : public Node {
public:
  Crowd() : Node("crowd") {
    srand(time(NULL));
    declare_parameter("update_frequency", 20.);
    double update_period_sec = 1.0 / get_parameter("update_frequency").as_double();

    SpawnData spawn_data = get_spawn_data();
    spawn_agents(spawn_data, agents_, people_);
    crowd_pub_ = create_publisher<PersonArray>("people", 10);
    marker_pub_ = create_publisher<MarkerArray>("people_markers", 10);

    auto update_period = duration<double>(update_period_sec);
    update_timer_ = create_wall_timer(update_period, [this, update_period] () {
      crowd_pub_->publish(people_);
    });
  }

private:
  shared_ptr<TimerBase> update_timer_; 
  shared_ptr<Publisher<PersonArray>> crowd_pub_;
  shared_ptr<Publisher<MarkerArray>> marker_pub_;

  std::vector<Agent> agents_;
  PersonArray people_;

  SpawnData get_spawn_data() {
    declare_parameter("num_agents", 5);
    declare_parameter("model.repel_strength", 1.);
    declare_parameter("model.avoid_radius", 1.5);
    declare_parameter("model.anisotropy", 0.5);
    declare_parameter("model.spawn_x_min", -10.);
    declare_parameter("model.spawn_y_min", -10.);
    declare_parameter("model.spawn_x_max", 10.);
    declare_parameter("model.spawn_y_max", 10.);

    int num_agents = get_parameter("num_agents").as_int();
    double repel_strength = get_parameter("model.repel_strength").as_double();
    double avoid_radius = get_parameter("model.avoid_radius").as_double();
    double anisotropy = get_parameter("model.anisotropy").as_double();
    double spawn_x_min = get_parameter("model.spawn_x_min").as_double();
    double spawn_y_min = get_parameter("model.spawn_y_min").as_double();
    double spawn_x_max = get_parameter("model.spawn_x_max").as_double();
    double spawn_y_max = get_parameter("model.spawn_y_max").as_double();

    return (SpawnData){
      .num_agents = num_agents,
      .avoid_radius = avoid_radius,
      .anisotropy = anisotropy,
      .spawn_x_min = spawn_x_min,
      .spawn_x_max = spawn_x_max,
      .spawn_y_min = spawn_y_min,
      .spawn_y_max = spawn_y_max
    };
  }

  void publish_markers() {
    MarkerArray markers;
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(make_shared<Crowd>());
  rclcpp::shutdown();
}