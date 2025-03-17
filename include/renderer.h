/**
 * @file renderer.h
 * @brief A renderer for the city
 */
#pragma once

#include <SFML/Graphics.hpp>

#include "cityGraph.h"
#include "cityMap.h"
#include "manager.h"

/**
 * @class Renderer
 * @brief A renderer for the city
 *
 * The renderer class is used to render the city map, the city graph and the cars.
 */
class Renderer {
public:
  /**
   * @brief Start the rendering
   */
  void startRender(const CityMap &cityMap, const CityGraph &cityGraph, Manager &manager);

  /**
   * @brief Render the city map
   * @param cityMap The city map
   */
  void renderCityMap(const CityMap &cityMap);

  /**
   * @brief Render the city graph
   * @param cityGraph The city graph
   * @param view The view
   */
  void renderCityGraph(const CityGraph &cityGraph, const sf::View &view);

  /**
   * @brief Render the cars
   * @param manager The manager
   */
  void renderManager(Manager &manager);

  /**
   * @brief Render the time
   */
  void renderTime();

  /**
   * @brief Render the conflicts
   */
  void setConflicts(const std::vector<AStar::conflict> &conflicts) { this->conflicts = conflicts; }

private:
  sf::RenderWindow window;
  double time;

  std::vector<AStar::conflict> conflicts;

  bool debug = false;
};

/**
 * @brief Draw an arrow
 * @param window The window
 * @param position The position
 * @param rotation The rotation
 * @param length The length
 * @param thickness The thickness
 * @param color The color
 * @param outline If the arrow should have an outline
 */
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
