#include "manager.h"
#include "renderer.h"
#include "utils.h"

#include <iostream>
#include <optional>
#include <spdlog/spdlog.h>

void Manager::createCarsCBS(int numCars) {
  this->createCarsAStar(numCars);

  std::priority_queue<CBSNode> openSet;

  CBSNode startNode;
  startNode.paths.resize(numCars);
  startNode.constraints.clear();
  startNode.constraints.resize(numCars);
  startNode.cost = 0;
  startNode.depth = 0;

  for (int i = 0; i < numCars; i++) {
    startNode.paths[i] = cars[i].getPath();
    startNode.constraints[i] = {};
    startNode.cost += cars[i].getRemainingTime(true);
  }

  openSet.push(startNode);

  // While there are conflicts in the paths, resolve them
  bool resolved = false;
  while (!openSet.empty()) {
    CBSNode node = openSet.top();
    openSet.pop();

    std::vector<std::vector<sf::Vector2f>> paths = node.paths;
    std::vector<std::vector<AStar::conflict>> constraints = node.constraints;
    double cost = node.cost;
    int depth = node.depth;

    int car1, car2;
    sf::Vector2f p1, p2;
    double a1, a2, time;
    bool conflict = hasConflict(paths, &car1, &car2, &p1, &p2, &a1, &a2, &time);

    if (!conflict) {
      spdlog::info("Resolved all conflicts");
      resolved = true;
      break;
    }

    spdlog::debug(
        "Conflict at Time: {:0>6.5} | Cost: {:0>6.5} | Depth: {:0>3} | Car1: {:0>3} | Car2: {:0>3} | Pos1: ({:0>5.4}, "
        "{:0>5.4}) | Pos2: ({:0>5.4}, {:0>5.4})",
        time, cost, depth, car1, car2, p1.x, p1.y, p2.x, p2.y);

    // Resolve conflict
    for (int iCar = 0; iCar < 2; iCar++) {
      int car = iCar == 0 ? car1 : car2;

      AStar::conflict newConflict;
      newConflict.point.position = iCar == 0 ? p2 : p1;
      newConflict.point.angle = iCar == 0 ? a2 : a1;
      newConflict.time = time;

      // If already in constraints, skip
      bool alreadyInConstraints = false;
      for (AStar::conflict conflict : node.constraints[car]) {
        if (conflict == newConflict) {
          alreadyInConstraints = true;
          break;
        }
      }
      if (alreadyInConstraints) {
        continue;
      }

      std::vector<AStar::conflict> newConstraints = node.constraints[car];
      newConstraints.push_back(newConflict);

      TimedAStar aStar(cars[car].getStart(), cars[car].getEnd(), graph, newConstraints);
      std::vector<AStar::node> newPath = aStar.findPath();

      if (newPath.empty()) {
        continue;
      }

      double carOldCost = cars[car].getRemainingTime(true);
      cars[car].assignPath(newPath);
      double carNewCost = cars[car].getRemainingTime(true);

      CBSNode newNode;
      newNode.paths = paths;
      newNode.paths[car] = cars[car].getPath();
      newNode.constraints = constraints;
      newNode.constraints[car] = newConstraints;
      newNode.cost = cost - carOldCost + carNewCost;
      newNode.depth = depth + 1;

      if (carOldCost > carNewCost) {
        spdlog::warn("Car {} old cost is higher than new cost, this is unexpected", car);
      }

      openSet.push(newNode);
    }
  }

  if (!resolved)
    spdlog::warn("Could not resolve all conflicts");
}

bool Manager::hasConflict(std::vector<std::vector<sf::Vector2f>> paths, int *car1, int *car2, sf::Vector2f *p1,
                          sf::Vector2f *p2, double *a1, double *a2, double *time) {
  int maxPathLength = 0;
  int numCars = (int)paths.size();
  for (int i = 0; i < numCars; i++) {
    maxPathLength = std::max(maxPathLength, (int)paths[i].size());
  }

  double width = graph.getWidth();
  double height = graph.getHeight();
  auto outOfBounds = [&](sf::Vector2f p) { return p.x < 0 || p.y < 0 || p.x > width || p.y > height; };

  for (int t = 0; t < maxPathLength; t++) {
    for (int i = 0; i < numCars; i++) {
      if (t >= (int)paths[i].size() - 1 || outOfBounds(paths[i][t]))
        continue;
      for (int j = i + 1; j < numCars; j++) {
        if (t >= (int)paths[j].size() - 1 || outOfBounds(paths[j][t]))
          continue;
        if (carsCollided(cars[i], cars[j], t)) {
          *car1 = i;
          *car2 = j;
          *p1 = paths[i][t];
          *p2 = paths[j][t];
          *a1 = std::atan2(paths[i][t + 1].y - paths[i][t].y, paths[i][t + 1].x - paths[i][t].x);
          *a2 = std::atan2(paths[j][t + 1].y - paths[j][t].y, paths[j][t + 1].x - paths[j][t].x);
          *time = (double)t * SIM_STEP_TIME;
          return true;
        }
      }
    }
  }

  return false;
}
