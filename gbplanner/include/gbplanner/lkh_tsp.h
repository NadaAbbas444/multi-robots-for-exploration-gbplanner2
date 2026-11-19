#pragma once
// Lin-Kernighan-Helsgaun TSP solver wrapper (ICUAS 2025, Sec. IV-B).
// Calls the external LKH-3 binary on a TSPLIB file. Falls back to
// nearest-neighbour + 2-opt if the binary is unavailable or fails.

#include <Eigen/Dense>
#include <string>
#include <vector>

namespace explorer {
namespace lkh {

// Solves a symmetric TSP over the full distance matrix `dist`.
// `tour` receives a permutation of 0..n-1 that starts at node `start`
// (the closing edge back to `start` is implied, not repeated).
// Returns true if LKH produced the tour, false if the fallback was used.
bool solveTsp(const Eigen::MatrixXd& dist, int start, std::vector<int>& tour,
              const std::string& lkh_binary = "LKH", int runs = 1,
              double time_limit_s = 2.0);

// Nearest-neighbour construction improved by 2-opt (fallback).
void solveTspGreedy2Opt(const Eigen::MatrixXd& dist, int start,
                        std::vector<int>& tour);

}  // namespace lkh
}  // namespace explorer
