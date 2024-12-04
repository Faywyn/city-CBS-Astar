#pragma once

#include <SFML/Graphics.hpp>
#include <vector>

#include "aStar.h"
#include "cityGraph.h"

class Car {
public:
  void assignPath(std::vector<AStar::node> path);
  void move();

  void render(sf::RenderWindow &window);

  // Getters
  float getSpeed();
  float getRemainingDistance();
  float getRemainingTime();
  float getElapsedTime();
  float getElapsedDistance();
  sf::Vector2f getPosition() { return path[currentPoint]; }

  // Setters
  void toggleDebug() { debug = !debug; }

private:
  std::vector<sf::Vector2f> path;
  int currentPoint = 0;
  bool debug = false;
};
