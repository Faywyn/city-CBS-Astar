/**
 * @file manager.h
 * @brief Manager for the cars
 *
 * This file contains the declaration of the Manager class. This class is used to manage the cars during the CBS
 * pathfinding. It creates the cars and resolves conflicts using the CBS algorithm.
 */
#pragma once

#include "dataManager.h"
#include "manager.h"
#include <SFML/Graphics.hpp>
#include <spdlog/spdlog.h>
#include <vector>

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

class ManagerCBS : public Manager {
public:
  using Node = _managerCBSNode;

  /**
   * @brief Constructor
   * @param cityGraph The city graph
   * @param CityMap The city map
   */
  ManagerCBS(const CityGraph &cityGraph, const CityMap &cityMap) : Manager(cityGraph, cityMap) {}

  /**
   * @brief Constructor in order to initialize a sub-CBS node
   * @param cityGraph The city graph
   * @param CityMap The city map
   * @param cars The cars
   */
  ManagerCBS(const CityGraph &cityGraph, const CityMap &cityMap, std::vector<Car> cars) : Manager(cityGraph, cityMap) {
    this->cars = cars;
    this->numCars = cars.size();
  }

  /**
   * @brief Using the created agents, create a path for each agent using an algorithm
   *
   * This function is used to create a path for each agent using an algorithm. The algorithm is not specified in this
   * class, but it is expected to be implemented in a derived class.
   */
  void planPaths() override {
    spdlog::info("Planning paths using CBS");
    createCarsCBS();
  }

private:
  /**
   * @brief Create cars using CBS pathfinding
   * @param numCars The number of cars
   * @return The data for the cars (success, data)
   */
  std::pair<bool, DataManager::data> createCarsCBS();

  /**
   * @brief Create a sub-CBS node
   * @param node The parent CBS node
   * @param subNodeDepth The depth of the sub-CBS node
   * @return The sub-CBS node
   *
   * This function creates a sub-CBS node from a parent CBS node. It creates a new node with the same paths and
   * constraints as the parent node, but with less agents.
   */
  Node createSubCBS(Node &node, int subNodeDepth);

  /**
   * @brief Process a CBS node
   * @param constraints The constraints
   * @param subNodeDepth The depth of the sub-CBS node
   * @return The processed CBS node
   *
   * This function processes a CBS node. It resolves conflicts and returns a new CBS node with the resolved conflicts.
   */
  Node processCBS(ConstraintController constraints, int subNodeDepth);

  /**
   * @brief Check if two cars have a conflict
   * @param paths The paths of the cars
   * @param car1 The first car
   * @param car2 The second car
   * @param p1 The position of the first car
   * @param p2 The position of the second car
   * @param a1 The angle of the first car using double
   * @param a2 The angle of the second car using double
   * @param time The time of the conflict
   * @return If the cars have a conflict
   */
  bool hasConflict(std::vector<std::vector<sf::Vector2f>> paths, int *car1, int *car2, sf::Vector2f *p1,
                   sf::Vector2f *p2, double *a1, double *a2, int *time);
};
