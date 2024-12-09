#include "manager.h"

#include <iostream>
#include <spdlog/spdlog.h>

void Manager::createCarsCBS(int numCars) {
  spdlog::info("Creating {} CBS cars", numCars);
  for (int i = 0; i < numCars; i++) {
    Car car;
    cars.push_back(car);
  }

  // Create a path for each car (random start and end points)
  for (int i = 0; i < numCars; i++) {
    Car &car = cars[i];
    std::vector<AStar::node> path;

    CityGraph::point start;
    CityGraph::point end;
    do {
      path.clear();
      start = graph.getRandomPoint();
      end = graph.getRandomPoint();

      sf::Vector2f diff = end.position - start.position;

      if (std::sqrt(std::pow(diff.x, 2) + std::pow(diff.y, 2)) < 100)
        continue;

      AStar aStar(start, end, graph);
      path = aStar.findPath();
    } while (path.empty() || (int)path.size() < 3);

    car.assignStartEnd(start, end);
    car.assignPath(path);
  }

  spdlog::info("Initial paths assigned to cars");

  std::priority_queue<CBSNode> openSet;

  CBSNode startNode;
  startNode.paths.resize(numCars);
  startNode.constraints.clear();
  startNode.cost = 0;
  startNode.depth = 0;

  for (int i = 0; i < numCars; i++) {
    startNode.paths[i] = cars[i].getPath();
    startNode.constraints.push_back({});
    startNode.cost += cars[i].getRemainingTime(true);
  }

  openSet.push(startNode);

  // While there are conflicts in the paths, resolve them
  while (!openSet.empty()) {
    CBSNode node = openSet.top();
    openSet.pop();

    std::vector<std::vector<sf::Vector2f>> paths = node.paths;
    std::vector<std::vector<AStar::conflict>> constraints = node.constraints;
    float cost = node.cost;
    int depth = node.depth;

    int car1, car2;
    sf::Vector2f p1, p2;
    float time;
    bool conflict = hasConflict(paths, &car1, &car2, &p1, &p2, &time);

    if (!conflict) {
      spdlog::info("Resolved all conflicts");
      break;
    }

    std::cout << "Conflict between cars " << car1 << " and " << car2 << " at " << p1.x << ", " << p1.y << " and "
              << p2.x << ", " << p2.y << " at time " << time << " | Cost: " << cost << " | Depth: " << depth
              << std::endl;

    // Resolve conflict
    for (int iCar = 0; iCar < 2; iCar++) {
      int car = iCar == 0 ? car1 : car2;
      float newCost = cost - cars[car].getRemainingTime(true);

      AStar::conflict newConflict;
      newConflict.position = iCar == 0 ? p2 : p1;
      newConflict.time = time;

      std::vector<AStar::conflict> newConstraints = node.constraints[car];
      newConstraints.push_back(newConflict);

      AStar aStar(cars[car].getStart(), cars[car].getEnd(), graph, newConstraints);
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
      newNode.cost = newCost + cars[car].getRemainingTime(true);
      newNode.depth = depth + 1;

      openSet.push(newNode);
    }
  }
}

bool Manager::hasConflict(std::vector<std::vector<sf::Vector2f>> paths, int *car1, int *car2, sf::Vector2f *p1,
                          sf::Vector2f *p2, float *time) {
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
        sf::Vector2f diff = paths[i][t] - paths[j][t];
        float distance = std::sqrt(diff.x * diff.x + diff.y * diff.y);
        if (distance < CAR_CBS_MIN_SPACING) {
          *car1 = i;
          *car2 = j;
          *p1 = paths[i][t];
          *p2 = paths[j][t];
          *time = t * SIM_STEP_TIME;
          return true;
        }
      }
    }
  }

  return false;
}
