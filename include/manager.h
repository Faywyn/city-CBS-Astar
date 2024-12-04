#pragma once

#include <SFML/Graphics.hpp>
#include <vector>

#include "car.h"
#include "cityGraph.h"

class Manager {
public:
  Manager(const CityGraph &cityGraph) : graph(cityGraph) {}

  void createCars(int numCars);
  void moveCars();
  void renderCars(sf::RenderWindow &window);

  void toggleCarDebug(sf::Vector2f mousePos);

private:
  std::vector<Car> cars;
  CityGraph graph;
};
