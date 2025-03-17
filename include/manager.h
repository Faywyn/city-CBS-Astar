/**
 * @file manager.h
 * @brief Manager for the cars
 *
 * This file contains the declaration of the Manager class. This class is used to manage the cars during the CBS
 * pathfinding. It creates the cars and resolves conflicts using the CBS algorithm.
 */
#pragma once

#include <SFML/Graphics.hpp>
#include <vector>

#include "car.h"
#include "cityGraph.h"
#include "dataManager.h"

/**
 * @struct _managerCBSNode
 * @brief A node for the CBS algorithm
 *
 * This struct represents a node for the CBS algorithm. It contains the paths for all agents, the constraints for all
 * agents, the individual path costs, the total cost, the depth in the CBS tree and if the node has resolved conflicts.
 */
typedef struct _managerCBSNode {
  std::vector<std::vector<sf::Vector2f>> paths; /**< \brief The paths for all agents */
  ConstraintController constraints;             /**< \brief The constraints for all agents */
  std::vector<double> costs;                    /**< \brief The individual path costs */
  double cost;                                  /**< \brief The total cost */
  int depth;                                    /**< \brief The depth in the CBS tree */
  bool hasResolved;                             /**< \brief If the node has resolved conflicts */

  bool operator<(const _managerCBSNode &other) const {
    return cost > other.cost || (cost == other.cost && depth > other.depth);
  }

} _managerCBSNode;

/**
 * @class Manager
 * @brief A manager for the cars
 *
 * The manager class is used to manage the cars during the CBS pathfinding. It creates the cars and resolves conflicts
 * using the CBS algorithm.
 */
class Manager {
public:
  using CBSNode = _managerCBSNode;

  /**
   * @brief Constructor
   * @param cityGraph The city graph
   * @param CityMap The city map
   * @param log If the manager should log
   */
  Manager(const CityGraph &cityGraph, const CityMap &CityMap, bool log) : graph(cityGraph), map(CityMap) {
    this->log = log;
  }

  /**
   * @brief Constructor
   * @param cityGraph The city graph
   * @param CityMap The city map
   * @param cars The cars
   * @param log If the manager should log
   */
  Manager(const CityGraph &cityGraph, const CityMap &CityMap, std::vector<Car> cars, bool log)
      : graph(cityGraph), map(CityMap), cars(cars) {
    this->numCars = cars.size();
    this->log = log;
  }

  /**
   * @brief Create cars using A* pathfinding, no collision avoidance
   * @param numCars The number of cars
   */
  void createCarsAStar(int numCars);

  /**
   * @brief Create cars using CBS pathfinding
   * @param numCars The number of cars
   * @return The data for the cars (success, data)
   */
  std::pair<bool, DataManager::data> createCarsCBS(int numCars);

  /**
   * @brief Create a sub-CBS node
   * @param node The parent CBS node
   * @param subNodeDepth The depth of the sub-CBS node
   * @return The sub-CBS node
   *
   * This function creates a sub-CBS node from a parent CBS node. It creates a new node with the same paths and
   * constraints as the parent node, but with less agents.
   */
  CBSNode createSubCBS(CBSNode &node, int subNodeDepth);

  /**
   * @brief Process a CBS node
   * @param constraints The constraints
   * @param subNodeDepth The depth of the sub-CBS node
   * @return The processed CBS node
   *
   * This function processes a CBS node. It resolves conflicts and returns a new CBS node with the resolved conflicts.
   */
  CBSNode processCBS(ConstraintController constraints, int subNodeDepth);

  /**
   * @brief Check if two cars have a conflict
   * @param paths The paths of the cars
   * @param car1 The first car
   * @param car2 The second car
   * @param p1 The position of the first car
   * @param p2 The position of the second car
   * @param a1 The angle of the first car
   * @param a2 The angle of the second car
   * @param time The time of the conflict
   * @return If the cars have a conflict
   */
  bool hasConflict(std::vector<std::vector<sf::Vector2f>> paths, int *car1, int *car2, sf::Vector2f *p1,
                   sf::Vector2f *p2, double *a1, double *a2, int *time);

  /**
   * @brief Move the cars to the next point in the path
   */
  void moveCars();

  /**
   * @brief Render the cars
   * @param window The window
   */
  void renderCars(sf::RenderWindow &window);

  /**
   * @brief Toggle the debug of one car
   * @param mousePos The mouse position
   *
   * This function toggles the debug of a car. If the mouse is over a car, the debug of the car is toggled.
   */
  void toggleCarDebug(sf::Vector2f mousePos);

  /**
   * @brief Get the number of cars
   * @return The number of cars
   */
  int getNumCars() { return numCars; }

  /**
   * @brief Get the cars
   * @return The cars
   */
  std::vector<Car> getCars() { return cars; }

private:
  int numCars;
  std::vector<Car> cars;
  CityGraph graph;
  CityMap map;
  bool log;
};
