#include "agent.hpp"

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

void spawn_agents_(SpawnData& spawn_data, std::vector<Agent>& agents, PersonArray& people) {
    double x_range = spawn_data.spawn_x_max - spawn_data.spawn_x_min;
    double y_range = spawn_data.spawn_y_max - spawn_data.spawn_y_min;
    for (int i = 0; i < spawn_data.num_agents; i++) {
      Agent agent;
      agent.radius = spawn_data.avoid_radius;
      agent.anisotropy = spawn_data.anisotropy;

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
        agent.goal.x() = (double)rand() / RAND_MAX * y_range + spawn_data.spawn_y_min;
        agent.goal.y() = spawn_data.spawn_y_min;
      }
      else {
        agent.goal.x() = (double)rand() / RAND_MAX * y_range + spawn_data.spawn_y_min;
        agent.goal.y() = spawn_data.spawn_y_min;
      }

      agents.push_back(agent);
      Person p = agent_to_person(agent);
      people.people.push_back(p);
    }
  }