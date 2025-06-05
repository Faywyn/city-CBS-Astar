/**
 * @file manager.h
 * @brief Manager for the cars
 *
 * This file contains the declaration of the Manager class. This class is used to manage the cars during the CBS
 * pathfinding. It creates the cars and resolves conflicts using the CBS algorithm.
 */
#pragma once

#include "car.h"
#include "cityGraph.h"
#include <SFML/Graphics.hpp>
#include <spdlog/spdlog.h>
#include <vector>

/**
 * @class Manager
 * @brief A manager for the cars
 *
 * The manager class is used to manage the cars during any pathfinding algorithm. It is used to create abstract managers
 * like a CBS one.
 */
class Manager {
public:
  /**
   * @brief Constructor
   * @param cityGraph The city graph
   * @param CityMap The city map
   */
  Manager(const CityGraph &cityGraph, const CityMap &CityMap) : graph(cityGraph), map(CityMap) {}

  /**
   * @brief Initialize agents and set up the system
   * @param numCars The number of agents
   */
  virtual void initializeAgents(int numAgents);

  /**
   * @brief Using the created agents, create a path for each agent using an algorithm
   *
   * This function is used to create a path for each agent using an algorithm. The algorithm is not specified in this
   * class, but it is expected to be implemented in a derived class.
   */
  virtual void planPaths() = 0;

  /**
   * @brief Make a simulation step
   */
  virtual void updateAgents();

  /**
   * @brief Process user input
   * @param event The event
   * @param window The window
   */
  virtual void userInput(sf::Event event, sf::RenderWindow &window) {};

  /**
   * @brief Render the agents based on their current position
   * @param window The window
   */
  virtual void renderAgents(sf::RenderWindow &window) final;

  /**
   * @brief Get the number of agents
   * @return The number of agents
   */
  virtual int getNumAgents() { return numCars; }

  /**
   * @brief Get the cars
   * @return The cars
   */
  virtual std::vector<Car> getCars() { return cars; }

protected:
  int numCars;
  std::vector<Car> cars;
  CityGraph graph;
  CityMap map;
};
