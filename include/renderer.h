#pragma once

#include <SFML/Graphics.hpp>

#include "cityGraph.h"
#include "cityMap.h"
#include "manager.h"

class Renderer {
public:
  void startRender(const CityMap &cityMap, const CityGraph &cityGraph, Manager &manager);
  void renderCityMap(const CityMap &cityMap);
  void renderCityGraph(const CityGraph &cityGraph, const sf::View &view);
  void renderManager(Manager &manager);
  void renderConflicts();
  void renderTime();

  void setConflicts(const std::vector<AStar::conflict> &conflicts) { this->conflicts = conflicts; }

private:
  sf::RenderWindow window;
  double time;

  std::vector<AStar::conflict> conflicts;

  bool debug = false;
};

inline void drawArrow(sf::RenderWindow &window, sf::Vector2f position, double rotation, double length, double thickness,
                      sf::Color color = sf::Color::Red, bool outline = false) {
  sf::ConvexShape arrow;

  arrow.setFillColor(color);
  arrow.setOrigin(-length / 2, 0);
  arrow.setPosition(position);
  arrow.setRotation(rotation);

  arrow.setPointCount(7);
  arrow.setPoint(0, sf::Vector2f(0, 0));
  arrow.setPoint(1, sf::Vector2f(-2 * length / 5, thickness));
  arrow.setPoint(2, sf::Vector2f(-2 * length / 5, thickness / 2));
  arrow.setPoint(3, sf::Vector2f(-length, thickness / 2));
  arrow.setPoint(4, sf::Vector2f(-length, -thickness / 2));
  arrow.setPoint(5, sf::Vector2f(-2 * length / 5, -thickness / 2));
  arrow.setPoint(6, sf::Vector2f(-2 * length / 5, -thickness));

  if (outline) {
    arrow.setOutlineThickness(thickness / 10);
    arrow.setOutlineColor(sf::Color::Black);
  }

  window.draw(arrow);
}
