#pragma once
#include "hungarian.hpp"
#include <Eigen/Dense>

using Eigen::Vector2d;

struct MatchResult {
    std::vector<std::pair<int, int>> matches;  // (old index, new index)
    std::vector<int> lost;                     // old indices with no match
    std::vector<int> born;                     // new indices with no match
};

inline MatchResult match(const std::vector<Vector2d>& old_p,
                         const std::vector<Vector2d>& new_p,
                         double d_max) {
    const int n = old_p.size(), m = new_p.size(), N = n + m;
    MatchResult res;
    if (N == 0) return res;

    const double BIG = 1e9;               // finite! inf breaks the arithmetic
    const double c = d_max * d_max / 2.0;  // cost of leaving one agent unmatched

    std::vector<std::vector<double>> M(N, std::vector<double>(N, 0.0));
    for (int i = 0; i < n; ++i)                       // top-left: real costs
        for (int j = 0; j < m; ++j) {
            double dx = old_p[i].x() - new_p[j].x(), dy = old_p[i].y() - new_p[j].y();
            M[i][j] = dx * dx + dy * dy;
        }
    for (int i = 0; i < n; ++i)                       // top-right: "old i lost"
        for (int k = 0; k < n; ++k) M[i][m + k] = (i == k) ? c : BIG;
    for (int k = 0; k < m; ++k)                       // bottom-left: "new j born"
        for (int j = 0; j < m; ++j) M[n + k][j] = (k == j) ? c : BIG;
    // bottom-right stays 0

    std::vector<int> a = hungarian(M);
    for (int r = 0; r < N; ++r) {
        int col = a[r];
        if (r < n && col < m) res.matches.push_back({r, col});
        else if (r < n)       res.lost.push_back(r);
        else if (col < m)     res.born.push_back(col);
    }
    return res;
}