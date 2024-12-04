#include "aStar.h"
#include "config.h"
#include "utils.h"

#include <ompl/base/State.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <spdlog/spdlog.h>

namespace ob = ompl::base;

AStar::AStar(graphPoint start, graphPoint end, const CityGraph &cityGraph) {
  this->start.position = start.position;
  this->start.angle = start.angle;
  this->end.position = end.position;
  this->end.angle = end.angle;
  this->graph = cityGraph;
}

void AStar::process() {
  path.clear();

  std::unordered_map<node, node> cameFrom;
  std::unordered_map<node, float> gScore;
  std::unordered_map<node, float> fScore;

  auto distance = [](const node &a, const node &b) {
    ob::DubinsStateSpace space(turningRadius(std::max(a.speed, b.speed)));
    ob::RealVectorBounds bounds(2);
    space.setBounds(bounds);

    ob::State *start = space.allocState();
    ob::State *end = space.allocState();

    start->as<ob::DubinsStateSpace::StateType>()->setXY(a.position.x, a.position.y);
    start->as<ob::DubinsStateSpace::StateType>()->setYaw(a.angle);

    end->as<ob::DubinsStateSpace::StateType>()->setXY(b.position.x, b.position.y);
    end->as<ob::DubinsStateSpace::StateType>()->setYaw(b.angle);

    return space.distance(start, end);
  };
  auto heuristic = [&](const node &a) { return distance(a, end) / CAR_MAX_SPEED_MS; };
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

    auto time = [](float distance, float prevSpeed, float speed) {
      float avgSpeed = (prevSpeed + speed) / 2;
      return distance / avgSpeed;
    };

    graphPoint currentGraphPoint = {current.position, current.angle};
    for (const auto &neighborGraphPoint : neighbors[currentGraphPoint]) {
      if (current.speed > neighborGraphPoint.maxSpeed)
        continue;

      float distance = neighborGraphPoint.distance;
      float distanceSpeed = std::sqrt(2 * CAR_ACCELERATION * distance);
      std::vector<float> speedChanges = {0, distanceSpeed, -distanceSpeed};

      if (current.speed + CAR_ACCELERATION > neighborGraphPoint.maxSpeed && current.speed < neighborGraphPoint.maxSpeed)
        speedChanges.push_back(neighborGraphPoint.maxSpeed - current.speed);

      for (const auto &speedChange : speedChanges) {
        float newSpeed = current.speed + speedChange;
        if (newSpeed > CAR_MAX_SPEED_MS)
          continue;

        if (newSpeed < 0.1)
          continue;

        node neighbor = {neighborGraphPoint.point.position, neighborGraphPoint.point.angle, newSpeed};
        float tentativeGScore = gScore[current] + time(distance, current.speed, newSpeed);

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
