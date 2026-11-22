#pragma once

#include "rclcpp/rclcpp.hpp"
#include "crowd_interfaces/msg/person.hpp"
#include "crowd_interfaces/msg/person_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <cstdint>
#include <memory>
#include <vector>
#include <Eigen/Dense>
#include "ros_gz_interfaces/srv/spawn_entity.hpp"
#include "ros_gz_interfaces/srv/set_entity_pose.hpp"
#include "ros_gz_interfaces/msg/entity.hpp"
#include "spawn_data.hpp"
#include "agent_data.hpp"

using std::shared_ptr;
using ros_gz_interfaces::srv::SpawnEntity;
using ros_gz_interfaces::srv::SetEntityPose;
using ros_gz_interfaces::msg::Entity;
using Eigen::Vector2d;

#define AGENT_IS_ROBOT(a) ((a).id == 0)

struct Agent {
  uint32_t id;
  Vector2d pos;
  Vector2d vel;
  Vector2d goal;
  double radius, avoid_radius;
  double anisotropy;
  double repel_strength;
};

crowd_interfaces::msg::Person agent_to_person(Agent& agent);

crowd_interfaces::msg::PersonArray agents_to_people(std::vector<Agent>& agents);

Agent create_robot_agent(AgentData data, Vector2d robot_pos);

void spawn_agents(SpawnData& spawn_data, AgentData& agent_data, std::vector<Agent>& agents);

void spawn_agents_gazebo(shared_ptr<rclcpp::Client<SpawnEntity>>& client,
                         const AgentData& agent_data, std::vector<Agent>& agents, size_t i = 0);

void change_agent_goal(SpawnData spawn_data, Agent& agent);

void update_agents(SpawnData spawn_data, AgentData agent_data, std::vector<Agent>& agents, double dt);

void update_agents_gazebo(shared_ptr<rclcpp::Client<SetEntityPose>>& client, 
                          const std::vector<Agent>& agents);

void publish_agent_markers(rclcpp::Time time, const AgentData& agent_data, std::vector<Agent>& agents,
                           std::shared_ptr<rclcpp::Publisher<visualization_msgs::msg::MarkerArray>> people_pub,
                           std::shared_ptr<rclcpp::Publisher<visualization_msgs::msg::MarkerArray>> arrow_pub);


std::string person_sdf(const std::string& name, double radius, double height);