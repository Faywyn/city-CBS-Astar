#include "aStar.h"
#include "config.h"
#include "dubins.h"
#include "utils.h"

#include <ompl/base/State.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <spdlog/spdlog.h>
#include <unordered_set>

namespace ob = ompl::base;

AStar::AStar(CityGraph::point start, CityGraph::point end, const CityGraph &cityGraph) {
  this->start.point = start;
  this->start.speed = 0;
  this->start.start = true;
  this->end.point = end;
  this->end.speed = 0;
  this->graph = cityGraph;
}

void AStar::process() {
  path.clear();

  std::unordered_map<AStar::node, AStar::node> cameFrom;
  std::unordered_map<AStar::node, double> gScore;
  std::unordered_map<AStar::node, double> fScore;

  auto heuristic = [&](const AStar::node &a) {
    CityGraph::neighbor end_;
    end_.point = end.point;
    end_.maxSpeed = 0;
    end_.turningRadius = CAR_MIN_TURNING_RADIUS;
    Dubins dubins(a.point, end_);
    return dubins.distance();
  };
  auto compare = [&](const AStar::node &a, const AStar::node &b) { return fScore[a] > fScore[b]; };

  std::priority_queue<AStar::node, std::vector<AStar::node>, decltype(compare)> openSet(compare);
  std::unordered_set<AStar::node> isInOpenSet;

  openSet.push(start);
  gScore[start] = 0;
  fScore[start] = heuristic(start);

  auto neighbors = graph.getNeighbors();

  int nbIterations = 0;
  while (!openSet.empty() && nbIterations++ < 1e5) {
    AStar::node current = openSet.top();
    openSet.pop();
    isInOpenSet.erase(current);

    if (current.point == end.point) {
      AStar::node currentCopy = current;

      while (!currentCopy.start) {
        path.push_back(currentCopy);
        currentCopy = cameFrom[currentCopy];
      }
      path.push_back(currentCopy);
      std::reverse(path.begin(), path.end());
      break;
    }

    for (const auto &neighborGraphPoint : neighbors[current.point]) {
      AStar::node neighbor;
      neighbor.point = neighborGraphPoint.point;
      neighbor.speed = neighborGraphPoint.maxSpeed;
      neighbor.arcFrom = {current.point, neighborGraphPoint};

      double tentativeGScore = gScore[current] + neighborGraphPoint.distance;

      if (gScore.find(neighbor) == gScore.end() || tentativeGScore < gScore[neighbor]) {
        cameFrom[neighbor] = current;
        gScore[neighbor] = tentativeGScore;
        fScore[neighbor] = gScore[neighbor] + heuristic(neighbor);

        if (isInOpenSet.find(neighbor) == isInOpenSet.end()) {
          openSet.push(neighbor);
          isInOpenSet.insert(neighbor);
        }
      }
    }
  }
}
