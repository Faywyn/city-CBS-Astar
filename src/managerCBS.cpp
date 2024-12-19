#include "manager.h"
#include "renderer.h"
#include "threadPool.h"

#include <iostream>
#include <optional>
#include <spdlog/spdlog.h>

// Structure used to store the result of a node processing (used in the thread pool)
struct NodeProcessingResult {
  bool solutionFound = false;
  double solutionCost;
  std::optional<Manager::CBSNode> solution;
  std::vector<Manager::CBSNode> newNodes;
};

// Function used to process a node in the CBS tree
NodeProcessingResult processNode(Manager::CBSNode node, Manager &manager, int numCars, CityGraph &graph) {
  NodeProcessingResult result;

  auto paths = node.paths;
  auto constraints = node.constraints;
  double cost = node.cost;
  int depth = node.depth;

  int car1, car2;
  sf::Vector2f p1, p2;
  double time;
  bool conflict = manager.hasConflict(paths, &car1, &car2, &p1, &p2, &time);

  spdlog::debug("Conflict at Time {} | Cost: {} | Depth: {}", time, cost, depth);

  if (!conflict) {
    result.solutionFound = true;
    result.solution = node;
    result.solutionCost = cost;
    return result;
  }

  // Resolve conflict
  for (int iCar = 0; iCar < 2; iCar++) {
    int car = iCar == 0 ? car1 : car2;

    AStar::conflict newConflict;
    newConflict.position = iCar == 0 ? p2 : p1;
    newConflict.time = time;

    bool alreadyInConstraints = false;
    for (auto &c : constraints[car]) {
      if (c.position == newConflict.position && c.time == newConflict.time) {
        alreadyInConstraints = true;
        break;
      }
    }
    if (alreadyInConstraints) {
      continue;
    }

    auto newConstraints = constraints[car];
    newConstraints.push_back(newConflict);

    TimedAStar aStar(node.starts[car], node.ends[car], graph, newConstraints);
    std::vector<AStar::node> newPath = aStar.findPath();

    if (newPath.empty()) {
      continue;
    }

    Manager::CBSNode newNode;
    newNode.paths = paths;
    newNode.paths[car] = newPath;
    newNode.constraints = constraints;
    newNode.constraints[car] = newConstraints;
    newNode.cost = 0;
    newNode.depth = depth + 1;
    newNode.starts = node.starts;
    newNode.ends = node.ends;

    for (int i = 0; i < numCars; i++) {
      Car _car = Car();
      _car.assignStartEnd(node.starts[i], node.ends[i]);
      _car.assignPath(newNode.paths[i]);
      newNode.cost += std::pow(_car.getRemainingTime(true), 2);
    }

    result.newNodes.push_back(std::move(newNode));
  }

  return result;
}

void Manager::createCarsCBS(int numCars) {
  this->createCarsAStar(numCars);

  std::priority_queue<CBSNode> openSet;

  CBSNode startNode;
  startNode.paths.resize(numCars);
  startNode.constraints.resize(numCars);
  startNode.cost = 0;
  startNode.depth = 0;
  startNode.starts.clear();
  startNode.ends.clear();

  for (int i = 0; i < numCars; i++) {
    startNode.paths[i] = cars[i].getAStarPath();
    startNode.constraints[i] = {};
    startNode.starts.push_back(cars[i].getStart());
    startNode.ends.push_back(cars[i].getEnd());
  }

  openSet.push(startNode);

  ThreadPool pool(std::thread::hardware_concurrency());

  bool resolved = false;
  std::mutex openSetMutex;

  while (!resolved) {
    std::vector<CBSNode> currentBatch;

    {
      std::unique_lock<std::mutex> lock(openSetMutex);
      if (openSet.empty()) {
        break;
      }

      size_t count = std::min(openSet.size(), pool.workersCount());
      for (size_t i = 0; i < count; i++) {
        currentBatch.push_back(openSet.top());
        openSet.pop();
      }
    }

    std::vector<std::future<NodeProcessingResult>> futures;
    futures.reserve(currentBatch.size());

    for (auto &node : currentBatch) {
      futures.push_back(pool.enqueue([&] { return processNode(node, *this, numCars, graph); }));
    }

    // Récupération des résultats
    for (auto &fut : futures) {
      NodeProcessingResult res = fut.get();
      if (res.solutionFound) {
        spdlog::info("Resolved all conflicts");
        resolved = true;

        for (int i = 0; i < numCars; i++) {
          cars[i].assignPath(res.solution->paths[i]);
        }
        break;
      } else {
        std::unique_lock<std::mutex> lock(openSetMutex);
        for (auto &newNode : res.newNodes) {
          openSet.push(std::move(newNode));
        }
      }
    }

    if (resolved) {
      break;
    }
  }

  if (!resolved)
    spdlog::warn("Could not resolve all conflicts");
}

bool Manager::hasConflict(std::vector<std::vector<AStar::node>> _paths, int *car1, int *car2, sf::Vector2f *p1,
                          sf::Vector2f *p2, double *time) {
  int maxPathLength = 0;
  int numCars = (int)_paths.size();
  std::vector<std::vector<sf::Vector2f>> paths;
  std::vector<Car> cars;

  for (int i = 0; i < numCars; i++) {
    Car car;
    cars.push_back(car);
    cars[i].assignPath(_paths[i]);
    paths.push_back(cars[i].getPath());
    maxPathLength = std::max(maxPathLength, (int)paths[i].size());
  }

  double width = graph.getWidth();
  double height = graph.getHeight();
  auto outOfBounds = [&](sf::Vector2f p) { return p.x < 0 || p.y < 0 || p.x >= width || p.y >= height; };

  for (int t = 0; t < maxPathLength; t++) {
    for (int i = 0; i < numCars; i++) {
      if (t >= (int)paths[i].size() || outOfBounds(paths[i][t]))
        continue;

      for (int j = i + 1; j < numCars; j++) {
        if (t >= (int)paths[j].size() || outOfBounds(paths[j][t]))
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
