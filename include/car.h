#pragma once

#include <SFML/Graphics.hpp>
#include <vector>

#include "aStar.h"
#include "cityGraph.h"

class Car {
public:
  Car();

  void assignStartEnd(CityGraph::point start, CityGraph::point end) {
    this->start = start;
    this->end = end;
  }
  void chooseRandomStartEndPath(CityGraph &graph, CityMap &cityMap);
  void assignPath(std::vector<AStar::node> path);
  void assignExistingPath(std::vector<sf::Vector2f> path);

  void move();
  void render(sf::RenderWindow &window);

  // Getters
  CityGraph::point getStart() { return start; }
  CityGraph::point getEnd() { return end; }
  double getSpeed();
  double getSpeedAt(int index);
  double getAverageSpeed(CityGraph &graph);

  double getRemainingTime();
  double getElapsedTime();
  double getPathTime();

  double getRemainingDistance();
  double getElapsedDistance();
  double getPathLength();

  sf::Vector2f getPosition() { return path[currentPoint]; }
  std::vector<sf::Vector2f> getPath() { return path; }
  std::vector<AStar::node> getAStarPath() { return aStarPath; }

  // Setters
  void toggleDebug() { debug = !debug; }

private:
  CityGraph::point start;
  CityGraph::point end;
  std::vector<sf::Vector2f> path;
  std::vector<AStar::node> aStarPath;
  int currentPoint = 0;
  bool debug = false;
  sf::Color color;
};
