#include "cityMap.h"
#include "config.h"

#include <ompl/base/State.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/DubinsStateSpace.h>

namespace ob = ompl::base;

inline sf::Vector2f latLonToXY(float lat, float lon) {
  sf::Vector2f xy;
  xy.x = EARTH_RADIUS * lon * M_PI / 180;
  xy.y = EARTH_RADIUS * std::log(std::tan((90.0f + lat) * M_PI / 360.0f));
  return xy;
}

inline float distance(sf::Vector2f p1, sf::Vector2f p2) {
  return std::sqrt(std::pow(p2.x - p1.x, 2) + std::pow(p2.y - p1.y, 2));
}

inline float normalizeAngle(float angle) { // -PI to PI
  while (angle > M_PI) {
    angle -= 2 * M_PI;
  }
  while (angle <= -M_PI) {
    angle += 2 * M_PI;
  }
  return angle;
}

inline float turningRadius(float speed) { return speed * speed / CAR_MAX_G_FORCE; }

sf::Font loadFont();
