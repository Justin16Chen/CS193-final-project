#pragma once
#include <vector>
#include <limits>

// Min-cost assignment. cost is n x m with n <= m.
// Returns assignment[i] = column assigned to row i.
inline std::vector<int> hungarian(const std::vector<std::vector<double>>& cost) {
    const int n = cost.size();
    if (n == 0) return {};
    const int m = cost[0].size();
    const double INF = std::numeric_limits<double>::infinity();

    // 1-indexed internally; index 0 is a sentinel.
    std::vector<double> u(n + 1, 0.0), v(m + 1, 0.0);   // dual potentials
    std::vector<int> p(m + 1, 0), way(m + 1, 0);        // p[j] = row matched to col j

    for (int i = 1; i <= n; ++i) {
        p[0] = i;
        int j0 = 0;
        std::vector<double> minv(m + 1, INF);
        std::vector<bool> used(m + 1, false);
        do {
            used[j0] = true;
            int i0 = p[j0], j1 = 0;
            double delta = INF;
            for (int j = 1; j <= m; ++j) {
                if (used[j]) continue;
                double cur = cost[i0 - 1][j - 1] - u[i0] - v[j];
                if (cur < minv[j]) { minv[j] = cur; way[j] = j0; }
                if (minv[j] < delta) { delta = minv[j]; j1 = j; }
            }
            for (int j = 0; j <= m; ++j) {
                if (used[j]) { u[p[j]] += delta; v[j] -= delta; }
                else         { minv[j] -= delta; }
            }
            j0 = j1;
        } while (p[j0] != 0);
        do {                       // walk the augmenting path back
            int j1 = way[j0];
            p[j0] = p[j1];
            j0 = j1;
        } while (j0 != 0);
    }

    std::vector<int> assignment(n, -1);
    for (int j = 1; j <= m; ++j)
        if (p[j] != 0) assignment[p[j] - 1] = j - 1;
    return assignment;
}