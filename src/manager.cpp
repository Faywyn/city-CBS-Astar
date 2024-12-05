#include "manager.h"
#include "aStar.h"

#include <iostream>
#include <spdlog/spdlog.h>

void Manager::createCars(int numCars) {
  spdlog::info("Creating {} cars", numCars);
  for (int i = 0; i < numCars; i++) {
    Car car;
    cars.push_back(car);
  }

  // Create a path for each car (random start and end points)
  for (int i = 0; i < numCars; i++) {
    Car &car = cars[i];
    std::vector<AStar::node> path;
    do {
      path.clear();

      CityGraph::point start = graph.getRandomPoint();
      CityGraph::point end = graph.getRandomPoint();

      if (std::sqrt(std::pow(start.position.x - end.position.x, 2) + std::pow(start.position.y - end.position.y, 2)) <
          100)
        continue;

      AStar aStar(start, end, graph);
      path = aStar.findPath();
    } while (path.empty() || (int)path.size() < 3);

    car.assignPath(path);
    spdlog::info("Car {} assigned path with {} points", i, path.size());
  }
}

void Manager::moveCars() {
  for (Car &car : cars) {
    car.move();
  }
}

void Manager::renderCars(sf::RenderWindow &window) {
  for (Car &car : cars) {
    car.render(window);
  }
}

void Manager::toggleCarDebug(sf::Vector2f mousePos) {
  for (Car &car : cars) {
    sf::Vector2f carPos = car.getPosition();
    float distance = sqrt(pow(mousePos.x - carPos.x, 2) + pow(mousePos.y - carPos.y, 2));
    if (distance < 5.0f) {
      car.toggleDebug();
      spdlog::debug("Toggled debug for car");
    }
  }
}
