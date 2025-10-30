/**
 * @file utils.h
 * @brief Utility functions for coordinate conversion, distance calculation, and collision detection
 */
#pragma once

#include "car.h"
#include "config.h"
#include <SFML/Graphics.hpp>

/**
 * @brief Convert geographic coordinates (latitude/longitude) to Cartesian coordinates (x/y)
 * 
 * Uses Web Mercator projection for conversion. Suitable for small-scale city maps.
 * 
 * @param lat The latitude in degrees
 * @param lon The longitude in degrees
 * @return Cartesian coordinates (x, y) in meters
 */
inline sf::Vector2f latLonToXY(const double lat, const double lon) {
  sf::Vector2f xy;
  xy.x = EARTH_RADIUS * lon * M_PI / 180.0;
  xy.y = EARTH_RADIUS * std::log(std::tan((90.0 + lat) * M_PI / 360.0));
  return xy;
}

/**
 * @brief Calculate Euclidean distance between two points
 * 
 * @param p1 The first point
 * @param p2 The second point
 * @return The distance in the same units as the input coordinates
 */
inline double distance(const sf::Vector2f p1, const sf::Vector2f p2) {
  const sf::Vector2f diff = p2 - p1;
  return std::sqrt(diff.x * diff.x + diff.y * diff.y);
}

/**
 * @brief Calculate the minimum turning radius for a given speed
 * 
 * Based on maximum lateral acceleration constraint (CAR_MAX_G_FORCE).
 * Uses the formula: r = v^2 / a_max
 * 
 * @param speed The speed in m/s
 * @return The minimum turning radius in meters
 */
inline double turningRadius(const double speed) { 
  return speed * speed / CAR_MAX_G_FORCE; 
}

/**
 * @brief Calculate the maximum speed for a given turning radius
 * 
 * Inverse of turningRadius function.
 * Uses the formula: v = sqrt(r * a_max)
 * 
 * @param radius The turning radius in meters
 * @return The maximum speed in m/s
 */
inline double turningRadiusToSpeed(const double radius) { 
  return std::sqrt(radius * CAR_MAX_G_FORCE); 
}

/**
 * @brief Load a font
 * @return The font
 */
sf::Font loadFont();
