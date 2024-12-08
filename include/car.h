#pragma once

#include <SFML/Graphics.hpp>
#include <vector>

#include "aStar.h"
#include "cityGraph.h"

class Car {
public:
  void assignStartEnd(CityGraph::point start, CityGraph::point end) {
    this->start = start;
    this->end = end;
  }
  void assignPath(std::vector<AStar::node> path);

  void move();
  void render(sf::RenderWindow &window);

  // Getters
  CityGraph::point getStart() { return start; }
  CityGraph::point getEnd() { return end; }
  float getSpeed();
  float getRemainingDistance();
  float getRemainingTime(bool fromStart = false);
  float getElapsedTime();
  float getElapsedDistance();
  sf::Vector2f getPosition() { return path[currentPoint]; }
  std::vector<sf::Vector2f> getPath() { return path; }

  // Setters
  void toggleDebug() { debug = !debug; }

private:
  CityGraph::point start;
  CityGraph::point end;
  std::vector<sf::Vector2f> path;
  int currentPoint = 0;
  bool debug = false;
};
