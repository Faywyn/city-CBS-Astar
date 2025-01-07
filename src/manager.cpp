#include "manager.h"
#include "aStar.h"

#include <iostream>
#include <spdlog/spdlog.h>

void Manager::createCarsAStar(int numCars) {
  spdlog::info("Creating {} AStar cars", numCars);
  for (int i = 0; i < numCars; i++) {
    Car car;
    cars.push_back(car);
  }

  // Create a path for each car (random start and end points)
  for (int i = 0; i < numCars; i++) {

    bool valid = false;
    do {
      cars[i].chooseRandomStartEndPath(graph, map);
      valid = true;
      for (int j = 0; j < i; j++) {
        sf::Vector2f diffStart = cars[j].getStart().position - cars[i].getStart().position;
        double distanceStart = std::sqrt(std::pow(diffStart.x, 2) + std::pow(diffStart.y, 2));
        if (distanceStart < 15) {
          cars[i].chooseRandomStartEndPath(graph, map);
          j = 0;
          valid = false;
          break;
        }
      }
    } while (!valid);

    spdlog::info("Car {} assigned path with {} points", i, cars[i].getPath().size());
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
    double distance = sqrt(pow(mousePos.x - carPos.x, 2) + pow(mousePos.y - carPos.y, 2));
    if (distance < 5.0f) {
      car.toggleDebug();
    }
  }
}
