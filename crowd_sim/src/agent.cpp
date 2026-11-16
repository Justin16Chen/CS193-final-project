#include "crowd_sim/agent.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "crowd_sim/rviz_helper.hpp"

#include <memory>
#include <algorithm>

#define ABS(x) ((x) < 0 ? -(x) : (x))
#define EPSILON (0.0001)

using Eigen::Vector2d;
using rclcpp::Publisher;
using visualization_msgs::msg::MarkerArray;
using visualization_msgs::msg::Marker;
using std::shared_ptr;

Person agent_to_person(Agent& agent) {
  Person p;
  p.id = agent.id;
  p.position.set__x(agent.pos.x());
  p.position.set__y(agent.pos.y());
  p.velocity.set__x(agent.vel.x());
  p.velocity.set__y(agent.vel.y());
  p.radius = agent.radius;
  return p;
}

PersonArray agents_to_people(std::vector<Agent>& agents) {
  PersonArray person_array;
  for (long unsigned int i = 0; i < agents.size(); i++) {
    person_array.people.push_back(agent_to_person(agents[i]));
  }
  return person_array;
}

void spawn_agents(SpawnData& spawn_data, AgentData &agent_data, std::vector<Agent>& agents) {
  static int id = 0;
  double x_range = spawn_data.spawn_x_max - spawn_data.spawn_x_min;
  double y_range = spawn_data.spawn_y_max - spawn_data.spawn_y_min;
  for (int i = 0; i < spawn_data.num_agents; i++) {
    Agent agent;
    agent.id = id++;
    agent.radius = agent_data.avoid_radius;
    agent.anisotropy = agent_data.anisotropy;
    agent.repel_strength = agent_data.repel_strength;

    double x = (double)rand() / RAND_MAX * x_range + spawn_data.spawn_x_min;
    double y = (double)rand() / RAND_MAX * y_range + spawn_data.spawn_y_min;
    agent.pos.x() = x;
    agent.pos.y() = y;

    double goal_rand = (double) rand() / RAND_MAX;
    if (goal_rand < 0.25) {
      agent.goal.x() = spawn_data.spawn_x_min;
      agent.goal.y() = (double)rand() / RAND_MAX * y_range + spawn_data.spawn_y_min;
    }
    else if (goal_rand < 0.5) {
      agent.goal.x() = spawn_data.spawn_x_max;
      agent.goal.y() = (double)rand() / RAND_MAX * y_range + spawn_data.spawn_y_min;
    }
    else if (goal_rand < 0.75) {
      agent.goal.x() = (double)rand() / RAND_MAX * x_range + spawn_data.spawn_x_min;
      agent.goal.y() = spawn_data.spawn_y_min;
    }
    else {
      agent.goal.x() = (double)rand() / RAND_MAX * x_range + spawn_data.spawn_x_min;
      agent.goal.y() = spawn_data.spawn_y_max;
    }
    agents.push_back(agent);
  }
}

void change_agent_goal(SpawnData spawn_data, Agent& agent) {
  double goal_rand = (double) rand() / RAND_MAX;
  // swap left to right and right to left
  if (ABS(agent.goal.x() - spawn_data.spawn_x_min) < EPSILON) {
    agent.goal.x() = spawn_data.spawn_x_max;
    agent.goal.y() = goal_rand * (spawn_data.spawn_y_max - spawn_data.spawn_y_min) + spawn_data.spawn_y_min;
  }
  else if (ABS(agent.goal.x() - spawn_data.spawn_x_max) < EPSILON) {
    agent.goal.x() = spawn_data.spawn_x_min;
    agent.goal.y() = goal_rand * (spawn_data.spawn_y_max - spawn_data.spawn_y_min) + spawn_data.spawn_y_min;
  }
  // swap top to bottom and bottom to top
  else if (ABS(agent.goal.y() - spawn_data.spawn_y_min) < EPSILON) {
    agent.goal.x() = goal_rand * (spawn_data.spawn_x_max - spawn_data.spawn_x_min) + spawn_data.spawn_x_min;
    agent.goal.y() = spawn_data.spawn_y_max;
  }
  else {
    agent.goal.x() = goal_rand * (spawn_data.spawn_x_max - spawn_data.spawn_x_min) + spawn_data.spawn_x_min;
    agent.goal.y() = spawn_data.spawn_y_min;
  }
}

// takes dt in seconds
void update_agents(SpawnData spawn_data, AgentData data, std::vector<Agent>& agents, double dt) {
  // calculate all accelerations
  std::vector<Vector2d> accels(agents.size(), Vector2d::Zero());
  for (size_t i = 0; i < agents.size(); i++) {
    Agent& a = agents[i];
    Vector2d agent_to_goal = a.goal - a.pos;
    if (agent_to_goal.norm() < 0.2) {
      change_agent_goal(spawn_data, a);
      agent_to_goal = a.goal - a.pos;
    }
    Vector2d goal_accel = Vector2d::Zero();
    goal_accel = (data.preferred_walk_speed * agent_to_goal.normalized() - a.vel) / data.relaxation_time;
    Vector2d agent_repulsion_accel = Vector2d::Zero();
    for (Agent& b : agents) {
      if (a.id == b.id) {
        continue;
      }
      Vector2d a_to_b = b.pos - a.pos;
      double avoid_dist = a.radius + b.radius - a_to_b.norm();
      double avoid_accel_mag = data.repel_strength * exp(avoid_dist / data.repel_strength_falloff);
      double cos_angle = a.vel.normalized().dot(a_to_b.normalized());
      double anisotrophy = data.anisotropy + (1 - data.anisotropy) * (1 + cos_angle) * 0.5;
      agent_repulsion_accel += avoid_accel_mag * -a_to_b.normalized() * anisotrophy;
    }
    accels[i] = goal_accel + agent_repulsion_accel;
  }

  // update velocity and position using acceleration
  for (size_t i = 0; i < agents.size(); i++) {
    Agent& a = agents[i];
    a.vel += accels[i] * dt;
    double mag = a.vel.norm();
    double new_mag = std::clamp(mag, -data.max_walk_speed, data.max_walk_speed);
    if (ABS(new_mag - mag) > 0.0001) {
      a.vel = a.vel.normalized() * new_mag;
    }
    a.pos += a.vel * dt;
  }
}

void publish_agent_markers(rclcpp::Time time, std::vector<Agent>& agents, shared_ptr<Publisher<MarkerArray>> people_pub,
                           shared_ptr<Publisher<MarkerArray>> arrow_pub) {
  MarkerArray people_markers;
  MarkerArray arrow_markers;
  for (Agent a : agents) {
    people_markers.markers.push_back(create_marker_cylinder(a.id, time, a.pos, 0.25, 1));
    arrow_markers.markers.push_back(create_marker_arrow(a.id, time, a.pos, a.goal));
  }
  people_pub->publish(people_markers);
  arrow_pub->publish(arrow_markers);
}