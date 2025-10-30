/**
 * @file utils.cpp
 * @brief Utility functions implementation
 */
#include "utils.h"
#include <spdlog/spdlog.h>

// Static variables for font caching
static bool fontLoaded = false;
static sf::Font font;

sf::Font loadFont() {
  if (!fontLoaded) {
    if (!font.openFromFile("assets/fonts/arial.ttf")) {
      spdlog::error("Failed to load font from assets/fonts/arial.ttf");
    }
    fontLoaded = true;
  }
  return font;
}

bool carsCollided(const Car car1, const Car car2, const int time) {
  const std::vector<sf::Vector2f> path1 = car1.getPath();
  const std::vector<sf::Vector2f> path2 = car2.getPath();
  
  // Validate time index is within bounds
  if (time < 0 || time >= static_cast<int>(path1.size()) || time >= static_cast<int>(path2.size())) {
    return false;
  }
  
  const sf::Vector2f diff = path1[time] - path2[time];
  const double dist = std::sqrt(diff.x * diff.x + diff.y * diff.y);
  return dist < CAR_LENGTH * COLLISION_SAFETY_FACTOR;
}

bool carConflict(const sf::Vector2f carPos, const sf::Angle carAngle, 
                 const sf::Vector2f confPos, const sf::Angle confAngle) {
  const sf::Vector2f diff = carPos - confPos;
  const double dist = std::sqrt(diff.x * diff.x + diff.y * diff.y);
  return dist < CAR_LENGTH * COLLISION_SAFETY_FACTOR;
}
