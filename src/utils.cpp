/**
 * @file utils.cpp
 * @brief Utility functions implementation
 */
#include "utils.h"
#include <spdlog/spdlog.h>

bool fontLoaded = false;
sf::Font font;

sf::Font loadFont() {
  if (!fontLoaded) {
    if (!font.openFromFile("assets/fonts/arial.ttf")) {
      spdlog::error("Failed to load font");
    }
    fontLoaded = true;
  }
  return font;
}

bool carsCollided(Car car1, Car car2, int time) {
  std::vector<sf::Vector2f> path1 = car1.getPath();
  std::vector<sf::Vector2f> path2 = car2.getPath();
  sf::Vector2f diff = path1[time] - path2[time];
  return std::sqrt(diff.x * diff.x + diff.y * diff.y) < CAR_LENGTH * COLLISION_SAFETY_FACTOR;
}

bool carConflict(sf::Vector2f carPos, sf::Angle carAngle, sf::Vector2f confPos, sf::Angle confAngle) {
  sf::Vector2f diff = carPos - confPos;
  return std::sqrt(diff.x * diff.x + diff.y * diff.y) < CAR_LENGTH * COLLISION_SAFETY_FACTOR;
}
