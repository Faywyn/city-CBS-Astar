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

AStar::AStar(CityGraph::point start, CityGraph::point end, const CityGraph &cityGraph,
             const std::vector<conflict> conflicts) {
  this->start.point = start;
  this->start.speed = 0;
  this->start.start = true;
  this->end.point = end;
  this->end.speed = 0;
  this->graph = cityGraph;
  this->conflicts = conflicts;
}

void AStar::process() {
  path.clear();

  std::unordered_map<node, node> cameFrom;
  std::unordered_map<node, float> gScore;
  std::unordered_map<node, float> fScore;

  auto heuristic = [&](const node &a) {
    CityGraph::neighbor end_;
    end_.point = end.point;
    end_.maxSpeed = 0;
    end_.turningRadius = CAR_MIN_TURNING_RADIUS;
    Dubins dubins(a.point, end_);
    return dubins.distance();
  };
  auto compare = [&](const node &a, const node &b) { return fScore[a] > fScore[b]; };

  std::priority_queue<node, std::vector<node>, decltype(compare)> openSet(compare);
  std::unordered_set<node> isInOpenSet;

  openSet.push(start);
  gScore[start] = 0;
  fScore[start] = heuristic(start);

  auto neighbors = graph.getNeighbors();

  int nbIterations = 0;
  while (!openSet.empty() && nbIterations++ < 10000) {
    node current = openSet.top();
    openSet.pop();
    isInOpenSet.erase(current);

    if (current.point == end.point) {
      node currentCopy = current;

      while (!currentCopy.start) {
        path.push_back(currentCopy);
        currentCopy = cameFrom[currentCopy];
      }
      path.push_back(currentCopy);
      std::reverse(path.begin(), path.end());
      break;
    }

    if (gScore[current] > 60 * 60 * 5)
      break;

    for (const auto &neighborGraphPoint : neighbors[current.point]) {
      if (current.speed > neighborGraphPoint.maxSpeed)
        continue;

      float distance = neighborGraphPoint.distance;
      float durationAcc =
          (-current.speed + std::sqrt(std::pow(current.speed, 2) + 2 * CAR_ACCELERATION * distance)) / CAR_ACCELERATION;
      float speedChangeAcc = CAR_ACCELERATION * durationAcc;
      float durationDec = (-neighborGraphPoint.maxSpeed +
                           std::sqrt(std::pow(neighborGraphPoint.maxSpeed, 2) - 2 * CAR_ACCELERATION * distance)) /
                          (-CAR_ACCELERATION);
      float speedChangeDec = -CAR_ACCELERATION * durationDec;
      std::vector<float> speedChanges = {0};
      if (std::isfinite(speedChangeAcc))
        speedChanges.push_back(speedChangeAcc);
      if (std::isfinite(speedChangeDec))
        speedChanges.push_back(speedChangeDec);

      if (std::isfinite(speedChangeAcc) && speedChangeAcc + current.speed > neighborGraphPoint.maxSpeed)
        speedChanges.push_back(neighborGraphPoint.maxSpeed - current.speed);
      if (std::isfinite(speedChangeDec) && speedChangeDec + current.speed < 0)
        speedChanges.push_back(-current.speed);

      if (distance == 0)
        speedChanges = {0};

      for (const auto &speedChange : speedChanges) {
        float newSpeed = current.speed + speedChange;
        if (newSpeed > CAR_MAX_SPEED_MS || newSpeed > neighborGraphPoint.maxSpeed)
          continue;

        if (newSpeed < 0)
          continue;

        node neighbor;
        neighbor.point = neighborGraphPoint.point;
        neighbor.speed = newSpeed;
        neighbor.arcFrom = {current.point, neighborGraphPoint};
        if (distance == 0) {
          if (gScore.find(neighbor) == gScore.end() || gScore[current] < gScore[neighbor]) {
            cameFrom[neighbor] = current;
            gScore[neighbor] = gScore[current];
            fScore[neighbor] = gScore[neighbor] + heuristic(neighbor);

            if (isInOpenSet.find(neighbor) == isInOpenSet.end()) {
              openSet.push(neighbor);
              isInOpenSet.insert(neighbor);
            }
          }
          continue;
        }

        float duration = distance / ((current.speed + newSpeed) / 2);
        float tentativeGScore = gScore[current] + duration;

        float t = gScore[current];
        bool isConflict = false;
        for (auto conflict : conflicts) {
          if (conflict.time < t - CAR_CBS_TIME_GAP || conflict.time > t + duration + CAR_CBS_TIME_GAP)
            continue;

          Dubins dubins(current.point, neighborGraphPoint, current.speed, newSpeed);
          float timeAbs = conflict.time - t;

          for (float t = timeAbs - CAR_CBS_TIME_GAP; t < timeAbs + CAR_CBS_TIME_GAP; t += SIM_STEP_TIME) {
            CityGraph::point point = dubins.point(t);
            float distance = std::sqrt(std::pow(point.position.x - conflict.position.x, 2) +
                                       std::pow(point.position.y - conflict.position.y, 2));
            if (distance < CAR_CBS_MIN_SPACING * 2) {
              isConflict = true;
              break;
            }
          }
          if (isConflict)
            break;
        }
        if (isConflict) {
          continue;
        }

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
}
