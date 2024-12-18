#include "manager.h"
#include "renderer.h"

#include <iostream>
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
    double time;
    bool conflict = hasConflict(paths, &car1, &car2, &p1, &p2, &time);

    if (!conflict) {
      spdlog::info("Resolved all conflicts");
      resolved = true;
      break;
    }

    std::cout << "Conflict between cars " << car1 << " and " << car2 << " at " << p1.x << ", " << p1.y << " and "
              << p2.x << ", " << p2.y << " at time " << time << " | Cost: " << cost << " | Depth: " << depth
              << std::endl;

    // Render
    // std::vector<AStar::conflict> renderConflicts;
    // renderConflicts.push_back({p1, time});
    // Renderer renderer;
    // renderer.setConflicts(renderConflicts);
    // renderer.startRender(map, graph, *this);

    // Resolve conflict
    for (int iCar = 0; iCar < 2; iCar++) {
      int car = iCar == 0 ? car1 : car2;

      AStar::conflict newConflict;
      newConflict.position = iCar == 0 ? p2 : p1;
      newConflict.time = time;

      // If already in constraints, skip
      bool alreadyInConstraints = false;
      for (AStar::conflict conflict : node.constraints[car]) {
        if (conflict.position == newConflict.position && conflict.time == newConflict.time) {
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

      cars[car].assignPath(newPath);
      CBSNode newNode;
      newNode.paths = paths;
      newNode.paths[car] = cars[car].getPath();
      newNode.constraints = constraints;
      newNode.constraints[car] = newConstraints;
      newNode.cost = 0;
      newNode.depth = depth + 1;

      // Recalculate cost
      for (int i = 0; i < numCars; i++) {
        newNode.cost += cars[i].getRemainingTime(true);
      }

      openSet.push(newNode);
    }
  }

  if (!resolved)
    spdlog::warn("Could not resolve all conflicts");
}

bool Manager::hasConflict(std::vector<std::vector<sf::Vector2f>> paths, int *car1, int *car2, sf::Vector2f *p1,
                          sf::Vector2f *p2, double *time) {
  int maxPathLength = 0;
  int numCars = (int)paths.size();
  for (int i = 0; i < numCars; i++) {
    maxPathLength = std::max(maxPathLength, (int)paths[i].size());
  }

  for (int t = 0; t < maxPathLength; t++) {
    for (int i = 0; i < numCars; i++) {
      if (t >= (int)paths[i].size())
        continue;
      for (int j = i + 1; j < numCars; j++) {
        if (t >= (int)paths[j].size())
          continue;
        if (cars[i].colidesWith(cars[j], double(t) * SIM_STEP_TIME)) {
          *car1 = i;
          *car2 = j;
          *p1 = paths[i][t];
          *p2 = paths[j][t];
          *time = (double)t * SIM_STEP_TIME;
          return true;
        }
      }
    }
  }

  return false;
}
