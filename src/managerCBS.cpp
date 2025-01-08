#include "manager.h"
#include "renderer.h"
#include "utils.h"

#include <iostream>
#include <numeric>
#include <optional>
#include <spdlog/spdlog.h>

void Manager::createCarsCBS(int numCars) {
  this->createCarsAStar(numCars);
  // std::cout << std::endl;

  std::priority_queue<CBSNode> openSet;

  CBSNode startNode;
  startNode.paths.resize(numCars);
  startNode.constraints.clear();
  startNode.constraints.resize(numCars);
  startNode.costs.clear();
  startNode.costs.resize(numCars);
  startNode.cost = 0;
  startNode.depth = 0;

  double maxCarCost = 0;

  for (int i = 0; i < numCars; i++) {
    startNode.paths[i] = cars[i].getPath();
    startNode.constraints[i] = {};

    double carCost = cars[i].getRemainingTime(true);
    startNode.costs[i] = carCost;
    startNode.cost += carCost;

    maxCarCost = std::max(maxCarCost, carCost);
  }

  openSet.push(startNode);
  int nbIterations = 0;
  int meanNbIterations = 20;

  std::vector<double> meanCosts;
  std::vector<double> meanDepths;
  std::vector<double> meanTimes;
  std::chrono::time_point<std::chrono::high_resolution_clock> start = std::chrono::high_resolution_clock::now();

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

      for (int i = 0; i < numCars; i++) {
        cars[i].assignExistingPath(paths[i]);
      }
      break;
    }

    meanCosts.push_back(cost);
    meanDepths.push_back(depth);
    meanTimes.push_back(time);

    if (nbIterations++ % meanNbIterations == 0) {
      double meanCost = std::accumulate(meanCosts.begin(), meanCosts.end(), 0.0) / meanCosts.size();
      double meanDepth = std::accumulate(meanDepths.begin(), meanDepths.end(), 0.0) / meanDepths.size();
      double meanTime = std::accumulate(meanTimes.begin(), meanTimes.end(), 0.0) / meanTimes.size();

      std::chrono::time_point<std::chrono::high_resolution_clock> end = std::chrono::high_resolution_clock::now();
      double duration = std::chrono::duration<double>(end - start).count();
      double remainingTime = (maxCarCost - meanTime) * (duration / meanTime);

      spdlog::info("Node cost: {:0>6.5} | Node depth: {:0>6.5} | Node conflic time: {:0>6.5} | Elapsed time: {}s | "
                   "Remaining time: ~{}s",
                   meanCost, meanDepth, meanTime, (int)duration, (int)remainingTime);
      std::cout << "\033[A\033[2K\r";

      meanCosts.clear();
      meanDepths.clear();
      meanTimes.clear();
    }

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

      cars[car].assignPath(newPath);
      double carOldCost = node.costs[car];
      double carNewCost = cars[car].getRemainingTime(true);

      CBSNode newNode;
      newNode.paths = paths;
      newNode.paths[car] = cars[car].getPath();
      newNode.constraints = constraints;
      newNode.constraints[car] = newConstraints;
      newNode.costs = node.costs;
      newNode.costs[car] = carNewCost;
      newNode.cost = cost - carOldCost + carNewCost;
      newNode.depth = depth + 1;

      // if (carOldCost > carNewCost) {
      //   std::cout << "\033[A\033[2K\r";
      //   spdlog::warn("Car {} old cost: {:0>6.5} | new cost: {:0>6.5}", car, carOldCost, carNewCost);
      // }

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

        sf::Vector2f diff = paths[i][t] - paths[j][t];
        if (std::sqrt(diff.x * diff.x + diff.y * diff.y) < CAR_LENGTH * 1.1) {
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
