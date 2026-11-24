#include "crowd_sim/path_planning.hpp"
#include "crowd_sim/match.hpp"
#include "crowd_sim/rviz_helper.hpp"
#include <assert.h>
#include <algorithm>

using Eigen::Vector3d;

std::vector<Vector2d> estimate_velocities(std::vector<Vector2d> old_positions,
                                          std::vector<Vector2d> new_positions,
                                          double dt, double d_max) {
    // one velocity per NEW position; unmatched (newly seen) agents default to zero
    std::vector<Vector2d> vel(new_positions.size(), Vector2d::Zero());

    MatchResult r = match(old_positions, new_positions, d_max);
    for (auto [i, j] : r.matches)
        vel[j] = (new_positions[j] - old_positions[i]) / dt;

    return vel;
}

vec2d_list smooth_velocities(const std::vector<vec2d_list> & prev_velocities,
                             const std::vector<double> & weights) {
    if (prev_velocities.empty()) return {};

    const vec2d_list & newest = prev_velocities.back();
    vec2d_list out(newest.size(), Vector2d::Zero());
    std::vector<double> weight_sum(newest.size(), 0.0);

    // Align the last weight with the newest frame; if there are more frames than
    // weights, the oldest frames are ignored.
    size_t n = std::min(prev_velocities.size(), weights.size());
    size_t frame0 = prev_velocities.size() - n;
    size_t weight0 = weights.size() - n;

    for (size_t k = 0; k < n; k++) {
        const vec2d_list & frame = prev_velocities[frame0 + k];
        double w = weights[weight0 + k];
        for (size_t i = 0; i < out.size() && i < frame.size(); i++) {
            out[i] += w * frame[i];
            weight_sum[i] += w;
        }
    }

    for (size_t i = 0; i < out.size(); i++) {
        if (weight_sum[i] > 0.0) out[i] /= weight_sum[i];
    }
    return out;
}


MarkerArray create_velocity_markers(std::vector<Vector2d> positions, 
  std::vector<Vector2d> velocities, rclcpp::Time time) {
  assert(positions.size() == velocities.size());
  MarkerArray markers;
  markers.markers.push_back(create_clear_marker("velocities", time));

  ColorRGBA color = make_color(0.2, 0.2, 0.9, 1.);
  for (unsigned long int i = 0; i < positions.size(); i++) {
    Vector3d pos1(positions[i].x(), positions[i].y(), 0);
    Vector2d end = positions[i] + velocities[i];
    Vector3d pos2(end.x(), end.y(), 0);
    markers.markers.push_back(create_marker_arrow("velocities", i + 1, time, pos1, pos2, color));
  }

  return markers;
}