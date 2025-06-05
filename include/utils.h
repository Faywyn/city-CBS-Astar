/**
 * @file utils.h
 * @brief Utility functions
 */
#pragma once

#include "car.h"
#include "config.h"
#include <SFML/Graphics.hpp>

/**
 * @brief Convert latitude and longitude to x and y
 * @param lat The latitude
 * @param lon The longitude
 * @return The x and y
 */
inline sf::Vector2f latLonToXY(double lat, double lon) {
  sf::Vector2f xy;
  xy.x = EARTH_RADIUS * lon * M_PI / 180;
  xy.y = EARTH_RADIUS * std::log(std::tan((90.0f + lat) * M_PI / 360.0f));
  return xy;
}

/**
 * @brief Get the distance between two points
 * @param p1 The first point
 * @param p2 The second point
 */
inline double distance(sf::Vector2f p1, sf::Vector2f p2) {
  return std::sqrt(std::pow(p2.x - p1.x, 2) + std::pow(p2.y - p1.y, 2));
}

/**
 * @brief Get the turning radius from the speed
 * @param speed The speed
 * @return The turning radius
 */
inline double turningRadius(double speed) { return speed * speed / CAR_MAX_G_FORCE; }

/**
 * @brief Get the speed from the turning radius
 * @param radius The turning radius
 * @return The speed
 */
inline double turningRadiusToSpeed(double radius) { return std::sqrt(radius * CAR_MAX_G_FORCE); }

/**
 * @brief Load a font
 * @return The font
 */
sf::Font loadFont();
