#pragma once

#include <vector>
#include <Eigen/Dense>
#include <cmath>

#define DEG2RAD(x) (M_PI * (x) / 180.)
#define RAD2DEG(x) (180. * (x) / M_PI)

typedef std::vector<Eigen::Vector2d> vec2d_list;
