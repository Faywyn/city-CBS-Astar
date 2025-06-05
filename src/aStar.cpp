/**
 * @file aStar.cpp
 * @brief A* algorithm implementation
 *
 * This file contains the implementation of the A* algorithm. It is used to find the shortest path between two points in
 * a graph.
 */
#include "aStar.h"
#include "config.h"
#include "dubins.h"
#include "utils.h"

#include <spdlog/spdlog.h>
#include <unordered_set>

AStar::AStar(CityGraph::point start, CityGraph::point end, const CityGraph &cityGraph) {
  this->start.point = start;
  this->start.speed = 0;
  this->end.point = end;
  this->end.speed = 0;
  this->graph = cityGraph;
}

void AStar::process() {
  std::unordered_map<AStar::node, AStar::node> cameFrom;
  std::unordered_map<AStar::node, double> gScore;
  std::unordered_map<AStar::node, double> fScore;

  auto heuristic = [&](const AStar::node &a) {
    sf::Vector2f diff = end.point.position - a.point.position;
    double distance = std::sqrt(diff.x * diff.x + diff.y * diff.y);
    return distance / CAR_MAX_SPEED_MS;
  };
  auto compare = [&](const AStar::node &a, const AStar::node &b) { return fScore[a] > fScore[b]; };

  std::priority_queue<AStar::node, std::vector<AStar::node>, decltype(compare)> openSetAstar(compare);
  std::unordered_set<AStar::node> isInOpenSet;

  openSetAstar.push(start);
  gScore[start] = 0;
  fScore[start] = heuristic(start);

  auto neighbors = graph.getNeighbors();

  int nbIterations = 0;
  while (!openSetAstar.empty() && nbIterations++ < 1e5) {
    AStar::node current = openSetAstar.top();
    openSetAstar.pop();
    isInOpenSet.erase(current);

    if (current.point == end.point) {
      AStar::node currentCopy = current;
      path.clear();

      while (!(currentCopy == start)) {
        path.push_back(currentCopy);
        currentCopy = cameFrom[currentCopy];
      }

      path.push_back(currentCopy);
      std::reverse(path.begin(), path.end());
      processed = true;
    }

    for (const auto &neighborGraphPoint : neighbors[current.point]) {
      if (current.speed > neighborGraphPoint.maxSpeed)
        continue;

      if (!neighborGraphPoint.isRightWay && ROAD_ENABLE_RIGHT_HAND_TRAFFIC)
        continue;

      std::vector<double> newSpeeds;
      newSpeeds.push_back(current.speed);

      double distance = graph.getInterpolator(current.point, neighborGraphPoint)->getDistance();
      double nSpeedAcc = std::sqrt(std::pow(current.speed, 2) + 2 * CAR_ACCELERATION * distance);
      double nSpeedDec = std::sqrt(std::pow(current.speed, 2) - 2 * CAR_DECELERATION * distance);

      auto push = [&](double nSpeed) {
        int numSpeedDiv = 5;
        for (int i = 1; i < numSpeedDiv + 1; i++) {
          double s = (current.speed + (nSpeed - current.speed) * i / numSpeedDiv);
          if (s < SPEED_RESOLUTION)
            continue;
          newSpeeds.push_back(s);
        }
      };

      if (nSpeedAcc > neighborGraphPoint.maxSpeed && current.speed < neighborGraphPoint.maxSpeed) {
        push(neighborGraphPoint.maxSpeed);
      } else if (nSpeedAcc < neighborGraphPoint.maxSpeed) {
        push(nSpeedAcc);
      }

      if (nSpeedDec == nSpeedDec && std::isfinite(nSpeedDec)) { // check if nSpeedDec is finite and not NaN
        if (nSpeedDec < 0 && current.speed > 0) {
          push(0);
        } else if (nSpeedDec >= 0) {
          push(nSpeedDec);
        }
      }

      AStar::node neighbor;
      neighbor.point = neighborGraphPoint.point;
      neighbor.arcFrom = {current.point, neighborGraphPoint};
      if (distance == 0) {
        neighbor.speed = current.speed;
        if (gScore.find(neighbor) == gScore.end() || gScore[current] < gScore[neighbor]) {
          cameFrom[neighbor] = current;
          gScore[neighbor] = gScore[current];
          fScore[neighbor] = gScore[neighbor] + heuristic(neighbor);

          if (isInOpenSet.find(neighbor) == isInOpenSet.end()) {
            openSetAstar.push(neighbor);

            isInOpenSet.insert(neighbor);
          }
        }
        continue;
      }

      for (const auto &newSpeed : newSpeeds) {
        if (newSpeed > CAR_MAX_SPEED_MS || newSpeed > neighborGraphPoint.maxSpeed || newSpeed < 0)
          continue;

        if (newSpeed == current.speed && newSpeed == 0)
          continue;

        neighbor.speed = newSpeed;

        double duration = 2 * distance / (current.speed + newSpeed);
        double tentativeGScore = gScore[current] + duration;

        double t = gScore[current];

        if (gScore.find(neighbor) == gScore.end() || tentativeGScore < gScore[neighbor]) {
          cameFrom[neighbor] = current;
          gScore[neighbor] = tentativeGScore;
          fScore[neighbor] = gScore[neighbor] + heuristic(neighbor);

          if (isInOpenSet.find(neighbor) == isInOpenSet.end()) {
            openSetAstar.push(neighbor);
            isInOpenSet.insert(neighbor);
          }
        }
      }
    }
  }
}
