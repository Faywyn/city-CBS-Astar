/**
 * @file managers/ocbs.cpp
 * @brief Optimal Conflict-Based Search (OCBS) implementation
 * 
 * This file contains the OCBS algorithm for multi-agent pathfinding. The pathfinding
 * method includes conflict checking, which differs from the basic A* in aStar.cpp.
 * 
 * @note The A* core logic is similar to aStar.cpp but includes additional conflict
 * checking for multi-agent coordination. This is intentional to keep conflict-aware
 * and conflict-free pathfinding separate.
 */
#include "aStar.h"
#include "config.h"
#include "dubins.h"
#include "manager_ocbs.h"
#include <spdlog/spdlog.h>

void ManagerOCBS::userInput(sf::Event event, sf::RenderWindow &window) {
  // If left mouse click over a car, toggle debug for that car
  if (event.is<sf::Event::MouseButtonPressed>() &&
      event.getIf<sf::Event::MouseButtonPressed>()->button == sf::Mouse::Button::Left) {
    sf::Vector2f mousePos = window.mapPixelToCoords(sf::Mouse::getPosition(window));
    for (int i = 0; i < numCars; i++) {
      sf::Vector2f diff = cars[i].getPosition() - mousePos;
      double len = std::sqrt(diff.x * diff.x + diff.y * diff.y);
      if (len < 2 * CAR_LENGTH) {
        cars[i].toggleDebug();
        spdlog::debug("Toggling debug for car {}", i);
        return;
      }
    }
  }
}

void ManagerOCBS::planPaths() {
  openSet = std::priority_queue<Node>();
  starts.clear();
  starts.resize(numCars);
  ends.clear();
  ends.resize(numCars);
  baseCosts.clear();
  baseCosts.resize(numCars);

  Node node;
  node.paths.resize(numCars);
  node.costs.resize(numCars);
  node.cost = 0;
  node.depth = 0;
  node.hasResolved = false;
  conflicts.clear();

  for (int i = 0; i < numCars; i++) {
    node.paths[i] = cars[i].getPath();
    node.costs[i] = cars[i].getPathTime();
    node.cost += node.costs[i];
    baseCosts[i] = node.costs[i];
    starts[i] = cars[i].getStart();
    ends[i] = cars[i].getEnd();
  }

  openSet.push(node);
  spdlog::info("Starting to find paths using CBS");
  findPaths();
}

void ManagerOCBS::initializePaths(Node *node) {
  for (int i = 0; i < numCars; i++) {
    spdlog::debug("Finding path for car {}", i);
    pathfinding(node, i);
  }
}

bool ManagerOCBS::findConflict(int *car1, int *car2, int *time, Node *node) {
  int maxPathLength = -1;
  for (int i = 0; i < numCars; i++) {
    maxPathLength = std::max(maxPathLength, (int)node->paths[i].size());
  }

  for (int t = 0; t < maxPathLength; t++) {
    for (int i = 0; i < numCars; i++) {
      for (int j = i + 1; j < numCars; j++) {
        sf::Vector2f diff = node->paths[i][t] - node->paths[j][t];
        double len = std::sqrt(diff.x * diff.x + diff.y * diff.y);
        if (len < CAR_LENGTH * COLLISION_SAFETY_FACTOR) {
          *car1 = i;
          *car2 = j;
          *time = t;
          return true;
        }
      }
    }
  }

  return false;
}

bool ManagerOCBS::findPaths() {
  if (openSet.empty()) {
    spdlog::info("No solution found");
    return false;
  }

  Node node = openSet.top();
  openSet.pop();

  spdlog::debug("Processing node with cost: {}", node.cost);

  int car1, car2, time;
  if (!findConflict(&car1, &car2, &time, &node)) {
    spdlog::info("Found solution with cost: {}", node.cost);

    for (int i = 0; i < numCars; i++) {
      cars[i].assignExistingPath(node.paths[i]);
    }

    return true;
  }

  spdlog::debug("Found conflict between car {} and car {} at time {}", car1, car2, time);

  // Witch car is the most affected
  int car1Index = car1;
  int car2Index = car2;

  double ratio1 = node.costs[car1] / baseCosts[car1];
  double ratio2 = node.costs[car2] / baseCosts[car2];

  if (ratio1 > ratio2) {
    car1Index = car2;
    car2Index = car1;
  }

  ConflictSituation situation1;
  situation1.car = car1Index;
  situation1.time = time * SIM_STEP_TIME;
  situation1.at = node.paths[car1Index][time];
  ConflictSituation situation2;
  situation2.car = car1Index;
  situation2.time = time * SIM_STEP_TIME;
  situation2.at = node.paths[car2Index][time];

  Conflict conflict1;
  conflict1.car = car1Index;
  conflict1.withCar = car2Index;
  conflict1.time = time * SIM_STEP_TIME;
  conflict1.position = node.paths[car1Index][time];

  Conflict conflict2;
  conflict2.car = car2Index;
  conflict2.withCar = car1Index;
  conflict2.time = time * SIM_STEP_TIME;
  conflict2.position = node.paths[car2Index][time];

  if (conflicts.find(situation1) == conflicts.end()) {
    conflicts[situation1] = new std::unordered_set<Conflict>();
  }
  conflicts[situation1]->insert(conflict1);

  if (conflicts.find(situation2) == conflicts.end()) {
    conflicts[situation2] = new std::unordered_set<Conflict>();
  }
  conflicts[situation2]->insert(conflict2);

  openSet.push(node);

  pathfinding(&node, car1Index);
  return findPaths();
}

void ManagerOCBS::pathfinding(Node *node, int carIndex) {
  AStar::node start;
  start.point = starts[carIndex];
  start.speed = 0;
  AStar::node end;
  end.point = ends[carIndex];
  end.speed = 0;

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
  while (!openSetAstar.empty() && nbIterations++ < ASTAR_MAX_ITERATIONS) {
    AStar::node current = openSetAstar.top();
    openSetAstar.pop();
    isInOpenSet.erase(current);

    if (current.point == end.point) {
      AStar::node currentCopy = current;
      std::vector<AStar::node> nodePaths;

      while (!(currentCopy == start)) {
        nodePaths.push_back(currentCopy);
        currentCopy = cameFrom[currentCopy];
      }

      nodePaths.push_back(currentCopy);
      std::reverse(nodePaths.begin(), nodePaths.end());

      double oldCost = node->costs[carIndex];
      cars[carIndex].assignPath(nodePaths, graph);

      node->paths[carIndex] = cars[carIndex].getPath();
      node->costs[carIndex] = cars[carIndex].getPathTime();
      node->cost += node->costs[carIndex] - oldCost;

      spdlog::debug("Found path for car {} with cost: {}", carIndex, node->costs[carIndex]);
      return;
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
        int numSpeedDiv = NUM_SPEED_DIVISIONS;
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
        bool conflictFree = true;

        // Checking for conflicts
        DubinsInterpolator *interpolator = graph.getInterpolator(current.point, neighborGraphPoint);
        for (double tt = 0; tt < duration; tt = tt + SIM_STEP_TIME) {
          ConflictSituation confS;
          confS.car = carIndex;
          confS.at = interpolator->get(tt, current.speed, newSpeed).position;
          confS.time = t + tt;

          if (conflicts.find(confS) == conflicts.end()) {
            continue;
          }

          std::unordered_set<Conflict> *conflictSet = conflicts[confS];

          if (conflictSet->size() == 0) {
            continue;
          }

          for (const auto &conf : *conflictSet) {
            // Check during all the duration if there is a conflict
            sf::Vector2f diff = confS.at - conf.position;
            double len = std::sqrt(diff.x * diff.x + diff.y * diff.y);

            if (len < CAR_LENGTH * COLLISION_SAFETY_FACTOR) {
              conflictFree = false;
              break;
            }
          }
          if (!conflictFree)
            break;
        }

        if (!conflictFree)
          continue;

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

  spdlog::warn("A* failed to find a path for car {}", carIndex);
}
