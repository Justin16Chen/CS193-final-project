#pragma once

struct AgentData {
  double person_radius;                       // physical body radius, used for drawing
  double person_height;                       // physical body height, used for drawing
  double avoid_radius, robot_avoid_radius;    // social-force personal-space radius
  double anisotropy;
  double sidestep;            // lateral steering strength relative to the radial repulsion
  double sidestep_cone;       // cos of half-angle: sidestep is 0 outside this cone, 1 dead-ahead
  double sidestep_lat_scale;  // lateral offset (sin of angle) at which the side choice saturates
  double sidestep_bias;       // tie-break when the neighbor is dead-ahead: +1 = pass on the right
  double repel_strength, repel_strength_falloff;
  double preferred_walk_speed, min_walk_speed, max_walk_speed;
  double relaxation_time;
};
