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

struct Agent {
  uint32_t id;
  Eigen::Vector2d pos;
  Eigen::Vector2d vel;
  Eigen::Vector2d goal;
  double radius;
  double anisotropy;
  double repel_strength;
};

struct SpawnData {
  int num_agents;
  double spawn_x_min, spawn_x_max;
  double spawn_y_min, spawn_y_max;
};

struct AgentData {
  double person_radius;   // physical body radius, used for drawing
  double avoid_radius;    // social-force personal-space radius
  double anisotropy;
  double repel_strength;
  double repel_strength_falloff;
  double preferred_walk_speed;
  double max_walk_speed;
  double relaxation_time;
};

crowd_interfaces::msg::Person agent_to_person(Agent& agent);

crowd_interfaces::msg::PersonArray agents_to_people(std::vector<Agent>& agents);

void spawn_agents(SpawnData& spawn_data, AgentData& agent_data, std::vector<Agent>& agents);

void change_agent_goal(SpawnData spawn_data, Agent& agent);

void update_agents(SpawnData spawn_data, AgentData agent_data, std::vector<Agent>& agents, double dt);

void publish_agent_markers(rclcpp::Time time, const AgentData& agent_data, std::vector<Agent>& agents,
                           std::shared_ptr<rclcpp::Publisher<visualization_msgs::msg::MarkerArray>> people_pub,
                           std::shared_ptr<rclcpp::Publisher<visualization_msgs::msg::MarkerArray>> arrow_pub);