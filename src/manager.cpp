/**
 * @file manager.cpp
 * @brief Implementation of the Manager class
 *
 * This file contains the implementation of the Manager class.
 */
#include "manager.h"
#include "aStar.h"

#include <iostream>
#include <spdlog/spdlog.h>

void Manager::initializeAgents(int numCars) {
  spdlog::info("Initializing {} agent...", numCars);

  for (int i = 0; i < numCars; i++) {
    Car car;
    cars.push_back(car);
  }

  // Create a path for each car (random start and end points)
  for (int i = 0; i < numCars; i++) {
    bool valid = false;
    cars[i].chooseRandomStartEndPath(graph, map);
  }

  spdlog::info("Initialized {} agents", cars.size());
}

void Manager::updateAgents() {
  for (Car &car : cars) {
    car.move();
  }
}

void Manager::renderAgents(sf::RenderWindow &window) {
  for (Car &car : cars) {
    car.render(window);
  }
}
