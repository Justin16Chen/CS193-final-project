#pragma once

struct SpawnData {
  int num_agents;
  bool spawn_left_right, spawn_top_bottom;
  double spawn_x_min, spawn_x_max;
  double spawn_y_min, spawn_y_max;
};
