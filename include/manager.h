#pragma once

#include <SFML/Graphics.hpp>
#include <vector>

#include "car.h"
#include "cityGraph.h"

typedef struct _managerCBSNode {
  std::vector<std::vector<sf::Vector2f>> paths;          // Paths for all agents
  std::vector<std::vector<AStar::conflict>> constraints; // Constraints for all agents
  double cost;                                           // Total cost (sum of individual path costs)
  int depth;                                             // Depth in the CBS tree

  // Comparator for priority queue (low cost has higher priority)
  bool operator<(const _managerCBSNode &other) const {
    return cost > other.cost || (cost == other.cost && depth < other.depth);
  }

} _managerCBSNode;

class Manager {
public:
  using CBSNode = _managerCBSNode;

  Manager(const CityGraph &cityGraph, const CityMap &CityMap) : graph(cityGraph), map(CityMap) {}

  void createCarsAStar(int numCars);
  void createCarsCBS(int numCars);
  void moveCars();
  void renderCars(sf::RenderWindow &window);

  void toggleCarDebug(sf::Vector2f mousePos);

private:
  bool hasConflict(std::vector<std::vector<sf::Vector2f>> paths, int *car1, int *car2, sf::Vector2f *p1,
                   sf::Vector2f *p2, double *time);

  std::vector<Car> cars;
  CityGraph graph;
  CityMap map;
};
