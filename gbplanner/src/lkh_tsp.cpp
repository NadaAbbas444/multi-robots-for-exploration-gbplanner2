#include "gbplanner/lkh_tsp.h"

#include <ros/ros.h>
#include <unistd.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <numeric>
#include <sstream>

namespace explorer {
namespace lkh {

namespace {

double tourLength(const Eigen::MatrixXd& d, const std::vector<int>& t) {
  double len = 0.0;
  for (size_t i = 0; i < t.size(); ++i) len += d(t[i], t[(i + 1) % t.size()]);
  return len;
}

void rotateToStart(std::vector<int>& tour, int start) {
  auto it = std::find(tour.begin(), tour.end(), start);
  if (it != tour.end()) std::rotate(tour.begin(), it, tour.end());
}

}  // namespace

void solveTspGreedy2Opt(const Eigen::MatrixXd& dist, int start,
                        std::vector<int>& tour) {
  const int n = dist.rows();
  tour.clear();
  if (n == 0) return;
  std::vector<bool> used(n, false);
  int cur = start;
  tour.push_back(cur);
  used[cur] = true;
  for (int k = 1; k < n; ++k) {
    int best = -1;
    double best_d = std::numeric_limits<double>::max();
    for (int j = 0; j < n; ++j)
      if (!used[j] && dist(cur, j) < best_d) best_d = dist(cur, j), best = j;
    tour.push_back(best);
    used[best] = true;
    cur = best;
  }
  // 2-opt on the closed tour; node 0 of the vector (start) stays fixed.
  bool improved = true;
  while (improved) {
    improved = false;
    for (int i = 1; i < n - 1; ++i) {
      for (int j = i + 1; j < n; ++j) {
        int a = tour[i - 1], b = tour[i], c = tour[j], e = tour[(j + 1) % n];
        double delta = dist(a, c) + dist(b, e) - dist(a, b) - dist(c, e);
        if (delta < -1e-9) {
          std::reverse(tour.begin() + i, tour.begin() + j + 1);
          improved = true;
        }
      }
    }
  }
}

bool solveTsp(const Eigen::MatrixXd& dist, int start, std::vector<int>& tour,
              const std::string& lkh_binary, int runs, double time_limit_s) {
  const int n = dist.rows();
  tour.clear();
  if (n <= 3) {  // trivial: any order is optimal for a closed tour
    tour.resize(n);
    std::iota(tour.begin(), tour.end(), 0);
    rotateToStart(tour, start);
    return true;
  }

  char dir_tmpl[] = "/tmp/gbplanner_lkh_XXXXXX";
  if (!mkdtemp(dir_tmpl)) {
    solveTspGreedy2Opt(dist, start, tour);
    return false;
  }
  const std::string dir(dir_tmpl);
  const std::string tsp = dir + "/p.tsp", par = dir + "/p.par",
                    out = dir + "/p.tour";

  // LKH needs integer weights: centimetres.
  {
    std::ofstream f(tsp);
    f << "NAME: inspection\nTYPE: TSP\nDIMENSION: " << n
      << "\nEDGE_WEIGHT_TYPE: EXPLICIT\nEDGE_WEIGHT_FORMAT: FULL_MATRIX\n"
         "EDGE_WEIGHT_SECTION\n";
    for (int i = 0; i < n; ++i) {
      for (int j = 0; j < n; ++j)
        f << (i == j ? 0 : static_cast<long>(std::lround(dist(i, j) * 100.0)))
          << (j + 1 < n ? " " : "\n");
    }
    f << "EOF\n";
  }
  {
    std::ofstream f(par);
    f << "PROBLEM_FILE = " << tsp << "\nTOUR_FILE = " << out
      << "\nRUNS = " << runs << "\nTIME_LIMIT = " << time_limit_s
      << "\nTRACE_LEVEL = 0\n";
  }

  const std::string cmd = lkh_binary + " " + par + " > /dev/null 2>&1";
  const int rc = std::system(cmd.c_str());

  bool ok = false;
  if (rc == 0) {
    std::ifstream f(out);
    std::string line;
    bool in_section = false;
    while (std::getline(f, line)) {
      if (line.rfind("TOUR_SECTION", 0) == 0) {
        in_section = true;
        continue;
      }
      if (!in_section) continue;
      int id = std::atoi(line.c_str());
      if (id == -1 || line.rfind("EOF", 0) == 0) break;
      if (id >= 1 && id <= n) tour.push_back(id - 1);
    }
    ok = (static_cast<int>(tour.size()) == n);
  }

  std::remove(tsp.c_str());
  std::remove(par.c_str());
  std::remove(out.c_str());
  rmdir(dir.c_str());

  if (!ok) {
    ROS_WARN_THROTTLE(10.0,
                      "[LKH] '%s' unavailable or failed (rc=%d); using "
                      "nearest-neighbour + 2-opt.",
                      lkh_binary.c_str(), rc);
    solveTspGreedy2Opt(dist, start, tour);
    return false;
  }
  rotateToStart(tour, start);
  ROS_INFO("[LKH] tour over %d viewpoints, length %.2f m", n,
           tourLength(dist, tour));
  return true;
}

}  // namespace lkh
}  // namespace explorer
