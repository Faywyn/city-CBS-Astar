/**
 * @file managers/index.cpp
 * @brief Implementation of the Manager class
 *
 * This file contains the base implementation of the Manager class which provides
 * common functionality for all pathfinding managers (CBS, OCBS, etc.).
 */
#include "manager.h"

void Manager::initializeAgents(int numCars) {
  spdlog::info("Initializing {} agent(s)...", numCars);
  this->numCars = numCars;

  // Reserve space to avoid reallocations
  cars.clear();
  cars.reserve(numCars);

  // Create car instances
  for (int i = 0; i < numCars; i++) {
    Car car;
    cars.push_back(car);
  }

  // Assign random start and end positions for each car
  for (int i = 0; i < numCars; i++) {
    cars[i].chooseRandomStartEndPath(graph, map);
  }

  spdlog::info("Successfully initialized {} agent(s)", cars.size());
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
