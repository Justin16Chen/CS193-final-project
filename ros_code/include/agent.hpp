#pragma once

#include "crowd_interfaces/msg/person.hpp"
#include "crowd_interfaces/msg/person_array.hpp"
#include <cstdint>
#include <Eigen/Dense>

using crowd_interfaces::msg::Person;
using crowd_interfaces::msg::PersonArray;

struct Agent {
  uint32_t id;
  Eigen::Vector2d pos;
  Eigen::Vector2d vel;
  Eigen::Vector2d goal;
  double radius;
  double anisotropy;
};

struct SpawnData {
  int num_agents;
  double avoid_radius;
  double anisotropy;
  double spawn_x_min, spawn_x_max;
  double spawn_y_min, spawn_y_max;
};

Person agent_to_person(const Agent& agent);

void spawn_agents(SpawnData& spawn_data, std::vector<Agent>& agents, PersonArray& people);