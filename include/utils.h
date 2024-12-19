#pragma once
#include "cityMap.h"
#include "config.h"

inline sf::Vector2f latLonToXY(double lat, double lon) {
  sf::Vector2f xy;
  xy.x = EARTH_RADIUS * lon * M_PI / 180;
  xy.y = EARTH_RADIUS * std::log(std::tan((90.0f + lat) * M_PI / 360.0f));
  return xy;
}

inline double distance(sf::Vector2f p1, sf::Vector2f p2) {
  return std::sqrt(std::pow(p2.x - p1.x, 2) + std::pow(p2.y - p1.y, 2));
}

inline double normalizeAngle(double angle) { // -PI to PI
  while (angle > M_PI) {
    angle -= 2 * M_PI;
  }
  while (angle <= -M_PI) {
    angle += 2 * M_PI;
  }
  return angle;
}

inline double turningRadius(double speed) { return speed * speed / CAR_MAX_G_FORCE; }

inline double turningRadiusToSpeed(double radius) { return std::sqrt(radius * CAR_MAX_G_FORCE); }

sf::Font loadFont();
