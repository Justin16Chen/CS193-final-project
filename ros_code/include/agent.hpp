#pragma once

#include "rclcpp/rclcpp.hpp"
#include "crowd_interfaces/msg/person.hpp"
#include "crowd_interfaces/msg/person_array.hpp"
#include "visualization_msgs/msg/marker.h"
#include "visualization_msgs/msg/marker_array.hpp"
#include <cstdint>
#include <Eigen/Dense>

using crowd_interfaces::msg::Person;
using crowd_interfaces::msg::PersonArray;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;
using rclcpp::Publisher;

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
  double avoid_radius;
  double anisotropy;
  double repel_strength;
  double repel_strength_falloff;
  double preferred_walk_speed;
  double max_walk_speed;
  double relaxation_time;
};

Person agent_to_person(const Agent& agent);

PersonArray agents_to_people(std::vector<Agent>& agents);

void spawn_agents(SpawnData& spawn_data, AgentData& agent_data, std::vector<Agent>& agents);

void update_agents(AgentData agent_data, std::vector<Agent>& agents, double dt);

void publish_agent_markers(rclcpp::Time time, std::vector<Agent>& agents, 
                           std::shared_ptr<Publisher<MarkerArray>> people_pub,
                           std::shared_ptr<Publisher<MarkerArray>> arrow_pub);