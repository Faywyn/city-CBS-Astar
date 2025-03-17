/**
 * @file utils.h
 * @brief Utility functions
 */
#pragma once
#include "config.h"
#include <SFML/Graphics.hpp>

class Car;

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
 * @brief Normalize an angle to -PI to PI
 * @param angle The angle
 */
inline double normalizeAngle(double angle) { // -PI to PI
  while (angle > M_PI) {
    angle -= 2 * M_PI;
  }
  while (angle <= -M_PI) {
    angle += 2 * M_PI;
  }
  return angle;
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
 * @bref Check if two cars collided
 * @param car1 The first car
 * @param car2 The second car
 */
bool carsCollided(Car car1, Car car2, int time);

/**
 * @brief Check if two cars have a conflict
 * @param carPos The position of the car
 * @param carAngle The angle of the car
 * @param confPos The position of the conflicting car
 * @param confAngle The angle of the conflicting car
 * @return If the cars have a conflict
 */
bool carConflict(sf::Vector2f carPos, double carAngle, sf::Vector2f confPos, double confAngle);

/**
 * @brief Load a font
 * @return The font
 */
sf::Font loadFont();
