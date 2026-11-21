#include "rclcpp/rclcpp.hpp"
#include "crowd_interfaces/msg/person_array.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "crowd_sim/agent.hpp"
#include "crowd_sim/rviz_helper.hpp"
#include "ros_gz_interfaces/srv/spawn_entity.hpp"
#include "nav_msgs/msg/odometry.hpp"

using rclcpp::Node;
using rclcpp::duration;
using rclcpp::TimerBase;
using rclcpp::Publisher;
using rclcpp::Subscription;
using std::shared_ptr;
using std::make_shared;
using crowd_interfaces::msg::PersonArray;
using crowd_interfaces::msg::Person;
using visualization_msgs::msg::MarkerArray;
using visualization_msgs::msg::Marker;
using ros_gz_interfaces::srv::SpawnEntity;
using nav_msgs::msg::Odometry;
using Eigen::Vector2d;

class Crowd : public Node {
public:
  Crowd() : Node("crowd") {
    srand(time(NULL));
    declare_parameter("update_frequency", 20.);
    double update_period_sec = 1.0 / get_parameter("update_frequency").as_double();

    crowd_pub_ = create_publisher<PersonArray>("people", 10);
    people_marker_pub_ = create_publisher<MarkerArray>("people_markers", 10);
    arrow_marker_pub_ = create_publisher<MarkerArray>("arrow_markers", 10);
    robot_imu_sub_ = create_subscription<Odometry>("model/robot/odometry", 10, [this] (const Odometry& odo) {
      robot_pos_.x() = odo.pose.pose.position.x;
      robot_pos_.y() = odo.pose.pose.position.y;
    });

    SpawnData spawn_data = get_spawn_data();
    AgentData agent_data = get_agent_data();
    spawn_agents(spawn_data, agent_data, agents_);
    spawn_client_ = create_client<SpawnEntity>("/world/crowd_world/create");
    spawn_client_->wait_for_service(std::chrono::seconds(10));
    spawn_agents_gazebo(spawn_client_, agent_data, agents_);

    set_pose_client_ = create_client<SetEntityPose>("/world/crowd_world/set_pose");
    set_pose_client_->wait_for_service(std::chrono::seconds(10));

    last_update_time_ = now();
    auto update_period = duration<double>(update_period_sec);
    update_timer_ = create_wall_timer(update_period, [this, spawn_data, agent_data] () {
      rclcpp::Time t = now();
      double dt = std::clamp((t - last_update_time_).seconds(), 0.0, kMaxDt);
      last_update_time_ = t;

      agents_.push_back(create_robot_agent(agent_data, robot_pos_)); // add the robot to the algorithm using its current position
      if (dt > 0.0) {
        update_agents(spawn_data, agent_data, agents_, dt);
        update_agents_gazebo(set_pose_client_, agents_);
      }
      PersonArray people = agents_to_people(agents_);
      crowd_pub_->publish(people);
      publish_agent_markers(t, agent_data, agents_, people_marker_pub_, arrow_marker_pub_);

      agents_.pop_back(); // remove the robot
    });
  }

private:
  static constexpr double kMaxDt = 0.1;  // seconds

  shared_ptr<TimerBase> update_timer_;
  rclcpp::Time last_update_time_;
  shared_ptr<Publisher<PersonArray>> crowd_pub_;
  shared_ptr<Publisher<MarkerArray>> people_marker_pub_;
  shared_ptr<Publisher<MarkerArray>> arrow_marker_pub_;
  shared_ptr<Subscription<Odometry>> robot_imu_sub_;

  Vector2d robot_pos_;
  std::vector<Agent> agents_;

  shared_ptr<rclcpp::Client<SpawnEntity>> spawn_client_;
  shared_ptr<rclcpp::Client<SetEntityPose>> set_pose_client_;

  SpawnData get_spawn_data() {
    declare_parameter("num_agents", 5);
    declare_parameter("crowd.spawn_left_right", true);
    declare_parameter("crowd.spawn_top_bottom", true);
    declare_parameter("crowd.spawn_x_min", -10.);
    declare_parameter("crowd.spawn_y_min", -10.);
    declare_parameter("crowd.spawn_x_max", 10.);
    declare_parameter("crowd.spawn_y_max", 10.);
    int num_agents = get_parameter("num_agents").as_int();
    bool spawn_left_right = get_parameter("crowd.spawn_left_right").as_bool();
    bool spawn_top_bottom = get_parameter("crowd.spawn_top_bottom").as_bool();
    double spawn_x_min = get_parameter("crowd.spawn_x_min").as_double();
    double spawn_y_min = get_parameter("crowd.spawn_y_min").as_double();
    double spawn_x_max = get_parameter("crowd.spawn_x_max").as_double();
    double spawn_y_max = get_parameter("crowd.spawn_y_max").as_double();
    return SpawnData{
      .num_agents = num_agents,
      .spawn_left_right = spawn_left_right,
      .spawn_top_bottom = spawn_top_bottom,
      .spawn_x_min = spawn_x_min,
      .spawn_x_max = spawn_x_max,
      .spawn_y_min = spawn_y_min,
      .spawn_y_max = spawn_y_max
    };
  }
  AgentData get_agent_data() {
    declare_parameter("sfm.person_radius", 0.2);
    declare_parameter("sfm.person_height", 1.7);
    declare_parameter("sfm.avoid_radius", 1.5);
    declare_parameter("sfm.robot_avoid_radius", 1.5);
    declare_parameter("sfm.anisotropy", 0.1);
    declare_parameter("sfm.sidestep", 0.2);
    declare_parameter("sfm.sidestep_cone", 0.7);
    declare_parameter("sfm.sidestep_lat_scale", 0.3);
    declare_parameter("sfm.sidestep_bias", 1.0);
    declare_parameter("sfm.repel_strength", 1.);
    declare_parameter("sfm.repel_strength_falloff", 0.3);
    declare_parameter("sfm.preferred_walk_speed", 1.);
    declare_parameter("sfm.min_walk_speed", 0.2);
    declare_parameter("sfm.max_walk_speed", 1.);
    declare_parameter("sfm.relaxation_time", 0.5);
    double repel_strength = get_parameter("sfm.repel_strength").as_double();
    double repel_strength_falloff = get_parameter("sfm.repel_strength_falloff").as_double();
    double anisotropy = get_parameter("sfm.anisotropy").as_double();
    double sidestep = get_parameter("sfm.sidestep").as_double();
    double sidestep_cone = get_parameter("sfm.sidestep_cone").as_double();
    double sidestep_lat_scale = get_parameter("sfm.sidestep_lat_scale").as_double();
    double sidestep_bias = get_parameter("sfm.sidestep_bias").as_double();
    double person_radius = get_parameter("sfm.person_radius").as_double();
    double person_height = get_parameter("sfm.person_height").as_double();
    double avoid_radius = get_parameter("sfm.avoid_radius").as_double();
    double robot_avoid_radius = get_parameter("sfm.robot_avoid_radius").as_double();
    double preferred_walk_speed = get_parameter("sfm.preferred_walk_speed").as_double();
    double min_walk_speed = get_parameter("sfm.min_walk_speed").as_double();
    double max_walk_speed = get_parameter("sfm.max_walk_speed").as_double();
    double relaxation_time = get_parameter("sfm.relaxation_time").as_double();
    return AgentData{
      .person_radius = person_radius,
      .person_height = person_height,
      .avoid_radius = avoid_radius,
      .robot_avoid_radius = robot_avoid_radius,
      .anisotropy = anisotropy,
      .sidestep = sidestep,
      .sidestep_cone = sidestep_cone,
      .sidestep_lat_scale = sidestep_lat_scale,
      .sidestep_bias = sidestep_bias,
      .repel_strength = repel_strength,
      .repel_strength_falloff = repel_strength_falloff,
      .preferred_walk_speed = preferred_walk_speed,
      .min_walk_speed = min_walk_speed,
      .max_walk_speed = max_walk_speed,
      .relaxation_time = relaxation_time
    };
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(make_shared<Crowd>());
  rclcpp::shutdown();
}