#include "crowd_sim/agent.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "crowd_sim/rviz_helper.hpp"
#include "ros_gz_interfaces/srv/spawn_entity.hpp"
#include "crowd_sim/utils.hpp"

#include <memory>
#include <algorithm>
#include <format>

constexpr double kEpsilon = 1e-4;

using Eigen::Vector2d;
using Eigen::Vector3d;
using crowd_interfaces::msg::Person;
using crowd_interfaces::msg::PersonArray;
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

enum class Side {
  TOP,
  LEFT,
  BOTTOM,
  RIGHT
};

void set_agent_goal(SpawnData& spawn_data, Agent& agent) {
  double goal_rand = (double) rand() / RAND_MAX;
  Side side;

  if (!spawn_data.spawn_left_right) {
    side = goal_rand < 0.5 ? Side::TOP : Side::BOTTOM;
  }
  else if (!spawn_data.spawn_top_bottom) {
    side = goal_rand < 0.5 ? Side::LEFT : Side::RIGHT;
  }
  else {
    if (goal_rand < 0.5) {
      side = goal_rand < 0.25 ? Side::TOP : Side::BOTTOM;
    }
    else {
      side = goal_rand < 0.75 ? Side::LEFT : Side::RIGHT;
    }
  }
  double x_range = spawn_data.spawn_x_max - spawn_data.spawn_x_min;
  double y_range = spawn_data.spawn_y_max - spawn_data.spawn_y_min;
  switch (side) {
    case Side::TOP:
      agent.goal.x() = (double)rand() / RAND_MAX * x_range + spawn_data.spawn_x_min;
      agent.goal.y() = spawn_data.spawn_y_min;
    break;
    case Side::BOTTOM:
      agent.goal.x() = (double)rand() / RAND_MAX * x_range + spawn_data.spawn_x_min;
      agent.goal.y() = spawn_data.spawn_y_max;
    break;
    case Side::LEFT:
      agent.goal.x() = spawn_data.spawn_x_min;
      agent.goal.y() = (double)rand() / RAND_MAX * y_range + spawn_data.spawn_y_min;
    break;
    case Side::RIGHT:
      agent.goal.x() = spawn_data.spawn_x_max;
      agent.goal.y() = (double)rand() / RAND_MAX * y_range + spawn_data.spawn_y_min;
    break;
  };
}

void spawn_agents(SpawnData& spawn_data, AgentData &agent_data, std::vector<Agent>& agents) {
  RCLCPP_INFO(rclcpp::get_logger("crowd"), "spawn_left_right: %d | spawn_top_bottom: %d\n",
              spawn_data.spawn_left_right, spawn_data.spawn_top_bottom);
  if (!spawn_data.spawn_left_right && !spawn_data.spawn_top_bottom)
    return;

  static int id = 1;
  double x_range = spawn_data.spawn_x_max - spawn_data.spawn_x_min;
  double y_range = spawn_data.spawn_y_max - spawn_data.spawn_y_min;
  for (int i = 0; i < spawn_data.num_agents; i++) {
    Agent agent{};
    agent.id = id++;
    agent.radius = agent_data.person_radius;
    agent.avoid_radius = agent_data.avoid_radius;
    agent.anisotropy = agent_data.anisotropy;
    agent.repel_strength = agent_data.repel_strength;

    double x = (double)rand() / RAND_MAX * x_range + spawn_data.spawn_x_min;
    double y = (double)rand() / RAND_MAX * y_range + spawn_data.spawn_y_min;
    agent.pos.x() = x;
    agent.pos.y() = y;

    set_agent_goal(spawn_data, agent);
    agents.push_back(agent);
  }
}

shared_ptr<SpawnEntity::Request> make_spawn_agent_gazebo_request(const Agent& agent, const AgentData& agent_data) {
  auto request = std::make_shared<SpawnEntity::Request>();
  request->entity_factory.name = std::format("person_{}", agent.id);
  request->entity_factory.sdf = person_sdf(request->entity_factory.name, agent_data.person_radius, agent_data.person_height);
  request->entity_factory.pose.position.x = agent.pos.x();
  request->entity_factory.pose.position.y = agent.pos.y();
  request->entity_factory.pose.position.z = 0.0;
  request->entity_factory.pose.orientation.w = 1.0;
  request->entity_factory.allow_renaming = false;
  return request;
}

void spawn_agents_gazebo(shared_ptr<rclcpp::Client<SpawnEntity>>& client, const AgentData& agent_data, std::vector<Agent>& agents, size_t i) {
  if (i >= agents.size()) {
    return;
  }
  auto request = make_spawn_agent_gazebo_request(agents[i], agent_data);
  client->async_send_request(request,
    // agent_data is copied: the caller's copy may be a constructor local that is gone by the time this fires
    [&client, agent_data, &agents, i] (rclcpp::Client<SpawnEntity>::SharedFuture f) {
      if (!f.get()->success) {
        RCLCPP_WARN(rclcpp::get_logger("crowd"), "spawn %zu failed", i);
      }
      spawn_agents_gazebo(client, agent_data, agents, i + 1);
    });
}

shared_ptr<SetEntityPose::Request> make_set_pose_request(const Agent& agent) {
  auto request = std::make_shared<SetEntityPose::Request>();
  request->entity.name = std::format("person_{}", agent.id);
  request->entity.type = Entity::MODEL;
  request->pose.position.x = agent.pos.x();
  request->pose.position.y = agent.pos.y();
  request->pose.position.z = 0.0;
  double yaw = agent.vel.norm() > kEpsilon ? std::atan2(agent.vel.y(), agent.vel.x()) : 0.0;
  request->pose.orientation.z = std::sin(yaw / 2.0);
  request->pose.orientation.w = std::cos(yaw / 2.0);
  return request;
}

void change_agent_goal(SpawnData spawn_data, Agent& agent) {
  double goal_rand = (double) rand() / RAND_MAX;
  // swap left to right and right to left
  if (ABS(agent.goal.x() - spawn_data.spawn_x_min) < kEpsilon) {
    agent.goal.x() = spawn_data.spawn_x_max;
    agent.goal.y() = goal_rand * (spawn_data.spawn_y_max - spawn_data.spawn_y_min) + spawn_data.spawn_y_min;
  }
  else if (ABS(agent.goal.x() - spawn_data.spawn_x_max) < kEpsilon) {
    agent.goal.x() = spawn_data.spawn_x_min;
    agent.goal.y() = goal_rand * (spawn_data.spawn_y_max - spawn_data.spawn_y_min) + spawn_data.spawn_y_min;
  }
  // swap top to bottom and bottom to top
  else if (ABS(agent.goal.y() - spawn_data.spawn_y_min) < kEpsilon) {
    agent.goal.x() = goal_rand * (spawn_data.spawn_x_max - spawn_data.spawn_x_min) + spawn_data.spawn_x_min;
    agent.goal.y() = spawn_data.spawn_y_max;
  }
  else {
    agent.goal.x() = goal_rand * (spawn_data.spawn_x_max - spawn_data.spawn_x_min) + spawn_data.spawn_x_min;
    agent.goal.y() = spawn_data.spawn_y_min;
  }
}

Agent create_robot_agent(AgentData data, Vector2d robot_pos) {
  return Agent{
    .id = 0,
    .pos = robot_pos,
    .vel = Vector2d::Zero(),
    .goal = Vector2d::Zero(),
    .radius = data.robot_avoid_radius,
    .avoid_radius = data.robot_avoid_radius,
    .anisotropy = 0,
    .repel_strength = 0
  };
}

// takes dt in seconds
void update_agents(SpawnData spawn_data, AgentData data, std::vector<Agent>& agents, double dt) {
  // calculate all accelerations
  std::vector<Vector2d> accels(agents.size(), Vector2d::Zero());
  for (size_t i = 0; i < agents.size(); i++) {
    Agent& a = agents[i];
    if (AGENT_IS_ROBOT(a))
      continue;
    Vector2d agent_to_goal = a.goal - a.pos;
    if (agent_to_goal.norm() < 0.2) {
      change_agent_goal(spawn_data, a);
      agent_to_goal = a.goal - a.pos;
    }
    Vector2d goal_accel = Vector2d::Zero();
    goal_accel = (data.preferred_walk_speed * agent_to_goal.normalized() - a.vel) / data.relaxation_time;
    // heading: velocity direction, or direction to goal when (nearly) stopped so a stuck
    // agent still knows which way is "forward" for anisotropy and sidestepping
    Vector2d heading = a.vel.norm() > kEpsilon ? a.vel.normalized() : agent_to_goal.normalized();
    Vector2d right(heading.y(), -heading.x());  // 90 deg clockwise from heading
    Vector2d agent_repulsion_accel = Vector2d::Zero();
    for (Agent& b : agents) {
      if (a.id == b.id) {
        continue;
      }
      Vector2d a_to_b = b.pos - a.pos;
      if (a_to_b.norm() < kEpsilon) {
        continue;  // coincident: no defined direction, skip rather than produce NaN
      }
      Vector2d a_to_b_hat = a_to_b.normalized();
      double avoid_dist = a.avoid_radius + b.avoid_radius - a_to_b.norm();
      double avoid_accel_mag = data.repel_strength * exp(avoid_dist / data.repel_strength_falloff);
      double cos_angle = heading.dot(a_to_b_hat);
      double anisotropy = data.anisotropy + (1 - data.anisotropy) * (1 + cos_angle) * 0.5;

      // sidestep magnitude: 0 outside the cone, ramps to 1 when the neighbor is dead-ahead
      double headon = std::clamp((cos_angle - data.sidestep_cone) / (1.0 - data.sidestep_cone), 0.0, 1.0);
      headon *= headon;  // remove the kink at the cone edge
      // sidestep direction: 2D cross product, +lat means neighbor is on the left -> steer right (f < 0 flips)
      double lat = heading.x() * a_to_b_hat.y() - heading.y() * a_to_b_hat.x();
      double f = std::clamp(-lat / data.sidestep_lat_scale, -1.0, 1.0);
      f += (1.0 - ABS(f)) * data.sidestep_bias;  // fill the dead-ahead tie with the bias

      Vector2d sidestep = data.sidestep * headon * f * right;
      agent_repulsion_accel += avoid_accel_mag * anisotropy * (-a_to_b_hat + sidestep);
    }
    accels[i] = goal_accel + agent_repulsion_accel;
  }

  // update velocity and position using acceleration
  for (size_t i = 0; i < agents.size(); i++) {
    Agent& a = agents[i];
    if (AGENT_IS_ROBOT(a))
      continue;
    a.vel += accels[i] * dt;
    double mag = a.vel.norm();
    double new_mag = std::clamp(mag, data.min_walk_speed, data.max_walk_speed);
    if (ABS(new_mag - mag) > 0.0001) {
      a.vel = a.vel.normalized() * new_mag;
    }
    a.pos += a.vel * dt;
  }
}

void update_agents_gazebo(shared_ptr<rclcpp::Client<SetEntityPose>>& client,
                          const std::vector<Agent>& agents) {
  for (const Agent& agent : agents) {
    if (AGENT_IS_ROBOT(agent))
      continue;
    auto request = make_set_pose_request(agent);
    client->async_send_request(request,
      [id = agent.id] (rclcpp::Client<SetEntityPose>::SharedFuture f) {
        if (!f.get()->success) {
          RCLCPP_WARN(rclcpp::get_logger("crowd"), "set_pose for person_%u rejected", id);
        }
      });
  }
}

void publish_agent_markers(rclcpp::Time time, const AgentData& agent_data, std::vector<Agent>& agents,
                           shared_ptr<Publisher<MarkerArray>> people_pub,
                           shared_ptr<Publisher<MarkerArray>> arrow_pub) {
  MarkerArray people_markers;
  MarkerArray arrow_markers;
  people_markers.markers.push_back(create_clear_marker("people", time));
  arrow_markers.markers.push_back(create_clear_marker("goals", time));
  for (Agent a : agents) {
    int r = AGENT_IS_ROBOT(a) ? 0 : 1;
    int g = AGENT_IS_ROBOT(a) ? 0 : 0;
    int b = AGENT_IS_ROBOT(a) ? 1 : 0;
    ColorRGBA color = make_color(r, g, b, 1);
    Vector3d agent_middle_pos = Vector3d(a.pos.x(), a.pos.y(), agent_data.person_height * 0.5);
    Vector3d agent_ground_pos = Vector3d(a.pos.x(), a.pos.y(), 0);
    Vector3d goal = Vector3d(a.goal.x(), a.goal.y(), 0);
    people_markers.markers.push_back(create_marker_cylinder("people", a.id, time, agent_middle_pos, a.radius, 
      agent_data.person_height, color));
    arrow_markers.markers.push_back(create_marker_arrow("goals", a.id, time, agent_ground_pos, goal, color));
  }
  people_pub->publish(people_markers);
  arrow_pub->publish(arrow_markers);
}

std::string person_sdf(const std::string& name, double radius, double height) {
  return std::format(R"(<?xml version="1.0"?>
<sdf version="1.10">
  <model name="{0}">
    <static>true</static>
    <link name="body">
      <pose>0 0 {3} 0 0 0</pose>
      <collision name="collision">
        <geometry><cylinder><radius>{1}</radius><length>{2}</length></cylinder></geometry>
      </collision>
      <visual name="visual">
        <geometry><cylinder><radius>{1}</radius><length>{2}</length></cylinder></geometry>
        <material><ambient>1 0 0 1</ambient><diffuse>1 0 0 1</diffuse></material>
      </visual>
    </link>
  </model>
</sdf>)", name, radius, height, height / 2);
}