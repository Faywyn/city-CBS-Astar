#include "aStar.h"
#include "config.h"
#include "dubins.h"
#include "utils.h"

#include <ompl/base/State.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <spdlog/spdlog.h>

namespace ob = ompl::base;

AStar::AStar(CityGraph::point start, CityGraph::point end, const CityGraph &cityGraph) {
  this->start.position = start.position;
  this->start.angle = start.angle;
  this->start.speed = 0;
  this->end.position = end.position;
  this->end.angle = end.angle;
  this->end.speed = 0;
  this->graph = cityGraph;
}

void AStar::process(std::unordered_set<conflict> conflicts) {
  path.clear();

  std::unordered_map<node, node> cameFrom;
  std::unordered_map<node, float> gScore;
  std::unordered_map<node, float> fScore;

  auto heuristic = [&](const node &a) {
    AStar::node end_ = end;
    end_.speed = -1;
    Dubins dubins(a, end_);
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

    if (current.position == end.position && current.angle == end.angle) {
      node currentCopy = current;
      while (cameFrom.find(currentCopy) != cameFrom.end()) {
        path.push_back(currentCopy);
        currentCopy = cameFrom[currentCopy];
      }
      path.push_back(start);
      std::reverse(path.begin(), path.end());
      break;
    }

    if (gScore[current] > 60 * 60 * 5)
      break;

    CityGraph::point currentGraphPoint = {current.position, current.angle};
    for (const auto &neighborGraphPoint : neighbors[currentGraphPoint]) {
      if (current.speed > neighborGraphPoint.maxSpeed)
        continue;

      float distance = neighborGraphPoint.distance;
      float distanceSpeed = std::sqrt(2 * CAR_ACCELERATION * distance);
      std::vector<float> speedChanges = {0, distanceSpeed, -distanceSpeed};

      if (current.speed + CAR_ACCELERATION > neighborGraphPoint.maxSpeed && current.speed < neighborGraphPoint.maxSpeed)
        speedChanges.push_back(neighborGraphPoint.maxSpeed - current.speed);

      if (distance == 0)
        speedChanges = {0};

      for (const auto &speedChange : speedChanges) {
        float newSpeed = current.speed + speedChange;
        if (newSpeed > CAR_MAX_SPEED_MS || newSpeed > neighborGraphPoint.maxSpeed)
          continue;

        if (newSpeed < 0)
          continue;

        node neighbor = {neighborGraphPoint.point.position, neighborGraphPoint.point.angle, newSpeed};
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

        float time = 2 * distance / (current.speed + newSpeed);
        float tentativeGScore = gScore[current] + time;

        float t = gScore[current];
        bool isConflict = false;
        for (auto conflict : conflicts) {
          if (conflict.time < t - CAR_CBS_TIME_GAP || conflict.time > t + time + CAR_CBS_TIME_GAP)
            continue;

          Dubins dubins(current, neighbor);
          CityGraph::point point = dubins.point(conflict.time - t);

          sf::Vector2f diff = point.position - conflict.position;

          if (std::sqrt(diff.x * diff.x + diff.y * diff.y) < CAR_CBS_MIN_SPACING * 5.0f) {
            isConflict = true;
            break;
          }
        }
        if (isConflict)
          continue;

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
