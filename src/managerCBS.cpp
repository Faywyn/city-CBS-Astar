/**
 * @file managerCBS.cpp
 * @brief CBS algorithm implementation
 *
 * This file contains the implementation of the CBS algorithm. It is used to resolve conflicts between cars.
 */
#include "managerCBS.h"
#include "priorityQueue.h"
#include "renderer.h"
#include "utils.h"

#include <iostream>
#include <numeric>
#include <spdlog/spdlog.h>

std::pair<bool, DataManager::data> ManagerCBS::createCarsCBS() {
  ConstraintController constraints;
  bool valid = true;

  Node node = processCBS(constraints, 0);
  if (!node.hasResolved) {
    spdlog::error("CBS could not resolve all conflicts");
    return std::make_pair(false, DataManager::data());
  } else {
    spdlog::info("CBS resolved all conflicts");
  }

  // Check if conflicts remain
  for (int i = 0; i < numCars; i++) {
    for (int j = i + 1; j < numCars; j++) {
      int tMin = std::min(cars[i].getPath().size(), cars[j].getPath().size());
      for (int t = 0; t < tMin; t++) {
        sf::Vector2f diff = cars[i].getPath()[t] - cars[j].getPath()[t];

        double width = graph.getWidth();
        double height = graph.getHeight();
        auto outOfBounds = [&](sf::Vector2f p) {
          return p.x + CAR_LENGTH < 0 || p.y + CAR_LENGTH < 0 || p.x > width + CAR_LENGTH || p.y > height + CAR_LENGTH;
        };

        if (outOfBounds(cars[i].getPath()[t]) || outOfBounds(cars[j].getPath()[t])) {
          continue;
        }

        if (std::sqrt(diff.x * diff.x + diff.y * diff.y) < CAR_LENGTH * 1.1) {
          spdlog::error("Cars {} and {} still have a conflict at time {} ({}, {})", i, j, t * SIM_STEP_TIME,
                        cars[i].getPath()[t].x, cars[i].getPath()[t].y);
          valid = false;
        }
      }
    }
  }

  if (!valid) {
    return std::make_pair(false, DataManager::data());
  }

  DataManager::data data;
  data.numCars = 0;
  data.carAvgSpeed.clear();

  for (int i = 0; i < numCars; i++) {
    double avgSpeed = cars[i].getAverageSpeed(graph);
    if (avgSpeed <= 0.01)
      continue;

    data.carAvgSpeed.push_back(avgSpeed);
    data.numCars++;
  }

  if (data.numCars == 0) {
    return std::make_pair(false, DataManager::data());
  }

  data.carDensity = 1000000 * data.numCars / (graph.getWidth() * graph.getHeight());

  return std::make_pair(true, data);
}

// Split the node into 2 subnodes
ManagerCBS::Node ManagerCBS::createSubCBS(Node &node, int subNodeDepth) {
  int numCars = (int)node.paths.size();
  int numCars1 = numCars / 2;
  int numCars2 = numCars - numCars1;

  std::vector<Car> cars1;
  std::vector<Car> cars2;

  std::vector<int> cars1Index;
  std::vector<int> cars2Index;

  for (int i = 0; i < numCars1; i++) {
    cars1.push_back(cars[i]);
    cars1Index.push_back(i);
  }
  for (int i = numCars1; i < numCars; i++) {
    cars2.push_back(cars[i]);
    cars2Index.push_back(i);
  }

  ConstraintController constraints1 = node.constraints.copy(cars1Index);
  ConstraintController constraints2 = node.constraints.copy(cars2Index);

  ManagerCBS manager1(graph, map, cars1);
  ManagerCBS manager2(graph, map, cars2);

  Node node1 = manager1.processCBS(constraints1, subNodeDepth + 1);
  if (!node1.hasResolved) {
    return node1;
  }

  // Push all manager1 cars pos to manager2 constraints
  for (int i = 0; i < numCars1; i++) {
    std::vector<sf::Vector2f> path = node1.paths[i];
    for (int j = 0; j < (int)path.size(); j += CBS_PRECISION_FACTOR) {
      AStar::conflict conflict;
      conflict.point.position = path[j];
      conflict.point.angle = sf::radians(0);
      conflict.time = j;

      if (conflict.point.position.x < -CAR_LENGTH || conflict.point.position.y < -CAR_LENGTH ||
          conflict.point.position.x > graph.getWidth() + CAR_LENGTH ||
          conflict.point.position.y > graph.getHeight() + CAR_LENGTH) {
        continue;
      }

      for (int k = 0; k < numCars2; k++) {
        conflict.car = k;
        constraints2.addConstraint(conflict);
      }
    }
  }

  Node node2 = manager2.processCBS(constraints2, subNodeDepth + 1);
  if (!node2.hasResolved) {
    return node2;
  }

  // Merge the 2 managers
  for (int i = 0; i < numCars1; i++) {
    node.costs[i] = node1.costs[i];
    node.paths[i] = node1.paths[i];
    cars[i].assignExistingPath(node1.paths[i]);
  }
  for (int i = numCars1; i < numCars; i++) {
    node.costs[i] = node2.costs[i - numCars1];
    node.paths[i] = node2.paths[i - numCars1];
    cars[i].assignExistingPath(node2.paths[i - numCars1]);
  }

  node.cost = node1.cost + node2.cost;
  node.depth = std::max(node1.depth, node2.depth);
  node.hasResolved = node1.hasResolved && node2.hasResolved;

  return node;
}

ManagerCBS::Node ManagerCBS::processCBS(ConstraintController constraints, int subNodeDepth) {
  PriorityQueue<Node> openSet = PriorityQueue<Node>(CBS_MAX_OPENSET_SIZE);

  Node startNode;
  startNode.paths.resize(numCars);
  startNode.constraints = constraints;
  startNode.costs.clear();
  startNode.costs.resize(numCars);
  startNode.cost = 0;
  startNode.depth = 0;
  startNode.hasResolved = false;

  double maxCarCost = 0;

  for (int i = 0; i < numCars; i++) {
    TimedAStar aStar(cars[i].getStart(), cars[i].getEnd(), graph, &constraints, i);
    std::vector<AStar::node> newPath = aStar.findPath();

    cars[i].assignPath(newPath);

    startNode.paths[i] = cars[i].getPath();

    double carCost = cars[i].getPathTime();
    startNode.costs[i] = carCost;
    startNode.cost += carCost;

    maxCarCost = std::max(maxCarCost, carCost);
  }

  openSet.push(startNode);

  // For logs
  std::vector<double> meanCosts;
  std::vector<double> meanDepths;
  std::vector<double> meanTimes;
  auto start = std::chrono::system_clock::now();
  double clockLastRefresh = 0;
  int numNodeProcessed = 0;

  // While there are conflicts in the paths, resolve them
  while (!openSet.empty()) {
    auto duration =
        std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - start).count() /
        1000.0;

    numNodeProcessed++;
    Node node = openSet.pop();

    // if (duration > CBS_MAX_SUB_TIME) {
    //   Node resSub = createSubCBS(node, subNodeDepth);
    //   if (resSub.hasResolved) {
    //     return resSub;
    //   }
    // }

    std::vector<std::vector<sf::Vector2f>> paths = node.paths;
    double cost = node.cost;
    int depth = node.depth;

    int car1, car2;
    sf::Vector2f p1, p2;

    double a1, a2;
    int time;
    bool conflict = hasConflict(paths, &car1, &car2, &p1, &p2, &a1, &a2, &time);

    if (!conflict) {
      for (int i = 0; i < numCars; i++) {
        cars[i].assignExistingPath(node.paths[i]);
      }
      node.hasResolved = true;
      return node;
    }

    meanCosts.push_back(cost);
    meanDepths.push_back(depth);
    meanTimes.push_back(time);

    if (clockLastRefresh + LOG_CBS_REFRESHRATE < duration) {
      double meanCost = std::accumulate(meanCosts.begin(), meanCosts.end(), 0.0) / meanCosts.size();
      double meanDepth = std::accumulate(meanDepths.begin(), meanDepths.end(), 0.0) / meanDepths.size();
      double meanTime = std::accumulate(meanTimes.begin(), meanTimes.end(), 0.0) / meanTimes.size();
      meanTime = meanTime * SIM_STEP_TIME;

      double remainingTime = (maxCarCost - meanTime) * (duration / meanTime);
      double processPerSecond = numNodeProcessed / (duration - clockLastRefresh);

      spdlog::info("Node C: {:0>6.5} | D: {:0>6.5} | CT: {:0>6.5} | SD: {} | ET: {}s | "
                   "ETR: ~{}s | Processed nodes: ~{:0>4.5}/s",
                   meanCost, meanDepth, meanTime, subNodeDepth, (int)duration, (int)remainingTime, processPerSecond);
      std::cout << "\033[A\033[2K\r";

      clockLastRefresh = duration;
      numNodeProcessed = 0;

      meanCosts.clear();
      meanDepths.clear();
      meanTimes.clear();
    }

    // Resolve conflict
    for (int iCar = 0; iCar < 2; iCar++) {
      int car = iCar == 0 ? car1 : car2;

      AStar::conflict newConflict;
      newConflict.point.position = iCar == 0 ? p2 : p1;
      newConflict.point.angle = iCar == 0 ? sf::radians(a2) : sf::radians(a1);
      newConflict.time = time;
      newConflict.car = iCar == 0 ? car1 : car2;

      // If already in constraints, skip
      if (node.constraints.hasConstraint(newConflict)) {
        continue;
      }

      ConstraintController newConstraints = node.constraints.copy();
      newConstraints.addConstraint(newConflict);

      TimedAStar aStar(cars[car].getStart(), cars[car].getEnd(), graph, &newConstraints, car);
      std::vector<AStar::node> newPath = aStar.findPath();

      if (newPath.empty()) {
        continue;
      }

      cars[car].assignPath(newPath);
      double carOldCost = node.costs[car];
      double carNewCost = cars[car].getPathTime();

      Node newNode;
      newNode.paths = paths;
      newNode.paths[car] = cars[car].getPath();
      newNode.constraints = newConstraints;
      newNode.costs = node.costs;
      newNode.costs[car] = carNewCost;
      newNode.cost = cost - carOldCost + carNewCost;
      newNode.depth = depth + 1;
      newNode.hasResolved = false;

      newNode.cost = 0;
      for (int i = 0; i < numCars; i++) {
        newNode.cost += newNode.costs[i];
      }

      openSet.push(newNode);
    }
  }

  return startNode;
}

bool ManagerCBS::hasConflict(std::vector<std::vector<sf::Vector2f>> paths, int *car1, int *car2, sf::Vector2f *p1,
                             sf::Vector2f *p2, double *a1, double *a2, int *time) {
  int maxPathLength = 0;
  int numCars = (int)paths.size();
  for (int i = 0; i < numCars; i++) {
    maxPathLength = std::max(maxPathLength, (int)paths[i].size());
  }

  double width = graph.getWidth();
  double height = graph.getHeight();
  auto outOfBounds = [&](sf::Vector2f p) {
    return p.x + CAR_LENGTH < 0 || p.y + CAR_LENGTH < 0 || p.x - CAR_LENGTH > width || p.y - CAR_LENGTH > height;
  };

  for (int t = 0; t < maxPathLength; t += CBS_PRECISION_FACTOR) {
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
          *time = t;
          return true;
        }
      }
    }
  }

  return false;
}
