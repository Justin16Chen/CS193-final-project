#include "rclcpp/rclcpp.hpp"
#include "crowd_interfaces/msg/person_array.hpp"

#include <stdlib.h>
#include <time.h>

using namespace rclcpp;
using namespace std;
using namespace crowd_interfaces::msg;

class Crowd : public Node {
public:
  Crowd() : Node("crowd") {
    srand(time(NULL));

    declare_parameter("num_people", 5);
    declare_parameter("update_frequency", 20.);
    declare_parameter("model.repel_strength", 1.);
    declare_parameter("model.avoid_radius", 1.5);
    declare_parameter("model.anisotropy", 0.5);
    declare_parameter("model.spawn_x_min", -10.);
    declare_parameter("model.spawn_y_min", -10.);
    declare_parameter("model.spawn_x_max", 10.);
    declare_parameter("model.spawn_y_max", 10.);

    int num_people = get_parameter("num_people").as_int();
    double update_period_sec = 1.0 / get_parameter("update_frequency").as_double();
    double repel_strength = get_parameter("model.repel_strength").as_double();
    double avoid_radius = get_parameter("model.avoid_radius").as_double();
    double anisotropy = get_parameter("model.anisotropy").as_double();
    double spawn_x_min = get_parameter("model.spawn_x_min").as_double();
    double spawn_y_min = get_parameter("model.spawn_y_min").as_double();
    double spawn_x_max = get_parameter("model.spawn_x_max").as_double();
    double spawn_y_max = get_parameter("model.spawn_y_max").as_double();

    spawn_people_(num_people, spawn_x_min, spawn_x_max, spawn_y_min, spawn_y_max);

    auto update_period = duration<double>(update_period_sec);
    update_timer_ = create_wall_timer(update_period, [this, update_period] () {
    });
  }
private:
  shared_ptr<TimerBase> update_timer_; 
  shared_ptr<Publisher<PersonArray>> crowd_pub_;

  PersonArray people_;

  void spawn_people_(int num_people, double spawn_x_min, double spawn_x_max, double spawn_y_min, double spawn_y_max) {
    double x_range = spawn_x_max - spawn_x_min;
    double y_range = spawn_y_max - spawn_y_min;
    for (int i = 0; i < num_people; i++) {
      Person p;
      double x = (double)rand() / RAND_MAX * x_range + spawn_x_min;
      double y = (double)rand() / RAND_MAX * y_range + spawn_y_min;
      p.position.set__x(x);
      p.position.set__y(y);
      people_.people.push_back(p);
    }

  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(make_shared<Crowd>());
  rclcpp::shutdown();
}