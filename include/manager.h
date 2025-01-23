#pragma once

#include <SFML/Graphics.hpp>
#include <vector>

#include "car.h"
#include "cityGraph.h"

typedef struct _managerCBSNode {
  std::vector<std::vector<sf::Vector2f>> paths; // Paths for all agents
  ConstraintController constraints;             // Constraints for all agents (global and local)
  std::vector<double> costs;                    // Individual path costs
  double cost;                                  // Total cost (sum of individual path costs)
  int depth;                                    // Depth in the CBS tree
  bool hasResolved;                             // Has resolved conflicts

  // Comparator for priority queue (low cost has higher priority)
  bool operator<(const _managerCBSNode &other) const {
    return cost > other.cost || (cost == other.cost && depth > other.depth);
  }

} _managerCBSNode;

class Manager {
public:
  using CBSNode = _managerCBSNode;

  Manager(const CityGraph &cityGraph, const CityMap &CityMap) : graph(cityGraph), map(CityMap) {}
  Manager(const CityGraph &cityGraph, const CityMap &CityMap, std::vector<Car> cars)
      : graph(cityGraph), map(CityMap), cars(cars) {
    this->numCars = cars.size();
  }

  // Simple A* pathfinding (no collision avoidance)
  void createCarsAStar(int numCars);

  // CBS pathfinding (collision avoidance)
  void createCarsCBS(int numCars);
  CBSNode createSubCBS(CBSNode &node, int subNodeDepth);
  CBSNode processCBS(ConstraintController constraints, int subNodeDepth);

  bool hasConflict(std::vector<std::vector<sf::Vector2f>> paths, int *car1, int *car2, sf::Vector2f *p1,
                   sf::Vector2f *p2, double *a1, double *a2, int *time);

  // Move and render cars
  void moveCars();
  void renderCars(sf::RenderWindow &window);
  void toggleCarDebug(sf::Vector2f mousePos);

  // Getters
  int getNumCars() { return numCars; }
  std::vector<Car> getCars() { return cars; }

private:
  int numCars;
  std::vector<Car> cars;
  CityGraph graph;
  CityMap map;
};
