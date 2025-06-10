/**
 * @file manager_ocbs.h
 * @brief Manager for the CBS algorithm
 */
#pragma once

#include "cityGraph.h"
#include "manager.h"
#include <SFML/Graphics.hpp>
#include <vector>

typedef struct _managerOCBSConflictSituation {
  int car;
  sf::Vector2f at;
  double time;

  bool operator==(const _managerOCBSConflictSituation &other) const {
    int t = std::round(time / OCBS_CONFLICT_RANGE);
    int oT = std::round(other.time / OCBS_CONFLICT_RANGE);
    int x = std::round(at.x / CELL_SIZE);
    int oX = std::round(other.at.x / CELL_SIZE);
    int y = std::round(at.y / CELL_SIZE);
    int oY = std::round(other.at.y / CELL_SIZE);
    return car == other.car && t == oT && x == oX && y == oY;
  }
} _managerOCBSConflictSituation;

typedef struct _managerOCBSConflict {
  int car;
  int withCar;
  double time;
  sf::Vector2f position;

  bool operator==(const _managerOCBSConflict &other) const {
    return car == other.car && withCar == other.withCar && time == other.time;
  }
} _managerOCBSConflict;

namespace std {
template <> struct hash<_managerOCBSConflictSituation> {
  std::size_t operator()(const _managerOCBSConflictSituation &point) const {
    int t = std::round(point.time / OCBS_CONFLICT_RANGE);
    int x = std::round(point.at.x / CAR_LENGTH);
    int y = std::round(point.at.y / CAR_LENGTH);
    return std::hash<int>()(point.car) ^ std::hash<int>()(t) ^ std::hash<int>()(x) ^ std::hash<int>()(y);
  }
};
template <> struct hash<_managerOCBSConflict> {
  std::size_t operator()(const _managerOCBSConflict &point) const {
    return std::hash<int>()(point.car) ^ std::hash<int>()(point.withCar) ^ std::hash<double>()(point.time) ^
           std::hash<float>()(point.position.x) ^ std::hash<float>()(point.position.y);
  }
};
} // namespace std

typedef struct _managerOCBSNode {
  std::vector<std::vector<sf::Vector2f>> paths; /**< \brief The paths for all agents */
  std::vector<double> costs;                    /**< \brief The individual path costs */
  double cost;                                  /**< \brief The total cost */
  int depth;                                    /**< \brief The depth in the CBS tree */
  bool hasResolved;                             /**< \brief If the node has resolved conflicts */
  // std::unordered_multimap<_managerOCBSConflictSituation, std::unordered_set<_managerOCBSConflict>>
  //     conflicts; /**< \brief The conflicts for all agents */

  bool operator<(const _managerOCBSNode &other) const {
    return cost > other.cost || (cost == other.cost && depth > other.depth);
  }
} _managerOCBSNode;

/**
 * @class ManagerOCBS
 * @brief Manager for the CBS algorithm
 * This class is responsible for managing the agents and their paths using the Conflict-Based Search (CBS) algorithm.
 * It inherits from the Manager class and implements the pathfinding logic specific to the CBS algorithm.
 * This class initializes paths for agents, handles user input, and plans paths using the CBS algorithm.
 */
class ManagerOCBS : public Manager {
public:
  using ConflictSituation = _managerOCBSConflictSituation;
  using Conflict = _managerOCBSConflict;
  using Node = _managerOCBSNode;

  /**
   * @brief Constructor
   * @param cityGraph The city graph
   * @param CityMap The city map
   */
  ManagerOCBS(const CityGraph &cityGraph, const CityMap &cityMap) : Manager(cityGraph, cityMap) {}

  /**
   * @brief Initialize agents and set up the system
   * @param numCars The number of agents
   */
  void initializePaths(Node *node);

  /**
   * @brief Make a simulation step
   */
  void userInput(sf::Event event, sf::RenderWindow &window) override;

  /**
   * @brief Using the created agents, create a path for each agent using an algorithm
   *
   * This function is used to create a path for each agent using an algorithm. The algorithm is not specified in this
   * class, but it is expected to be implemented in a derived class.
   */
  void planPaths() override;

private:
  bool findConflict(int *car1, int *car2, int *time, Node *node);
  bool findPaths();
  void pathfinding(Node *node, int carIndex);

  std::vector<_cityGraphPoint> starts;           /**< \brief The start points of the cars */
  std::vector<_cityGraphPoint> ends;             /**< \brief The end points of the cars */
  std::vector<double> baseCosts;                 /**< \brief The base costs of the cars */
  std::priority_queue<_managerOCBSNode> openSet; /**< \brief The open set for the CBS algorithm */
  std::unordered_map<_managerOCBSConflictSituation, std::unordered_set<_managerOCBSConflict> *>
      conflicts; /**< \brief The conflicts for all agents */
};
