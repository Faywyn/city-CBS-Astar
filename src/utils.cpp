/**
 * @file utils.cpp
 * @brief Utility functions implementation
 */
#include <spdlog/spdlog.h>

#include "car.h"
#include "utils.h"

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
  return std::sqrt(diff.x * diff.x + diff.y * diff.y) < CAR_LENGTH * 1.1;

  sf::Vector2f pos1 = path1[time];
  sf::Vector2f pos2 = path2[time];

  double angle1 = atan2(path1[time + 1].y - path1[time].y, path1[time + 1].x - path1[time].x);
  double angle2 = atan2(path2[time + 1].y - path2[time].y, path2[time + 1].x - path2[time].x);

  sf::Vector2f p11 = pos1 + sf::Vector2f(CAR_LENGTH / 2.0f * cos(angle1), CAR_LENGTH / 2.0f * sin(angle1));
  sf::Vector2f p12 = pos1 - sf::Vector2f(CAR_LENGTH / 2.0f * cos(angle1), CAR_LENGTH / 2.0f * sin(angle1));
  sf::Vector2f p21 = pos2 + sf::Vector2f(CAR_LENGTH / 2.0f * cos(angle2), CAR_LENGTH / 2.0f * sin(angle2));
  sf::Vector2f p22 = pos2 - sf::Vector2f(CAR_LENGTH / 2.0f * cos(angle2), CAR_LENGTH / 2.0f * sin(angle2));

  bool colides = false;
  colides |= std::sqrt(std::pow(p11.x - p21.x, 2) + std::pow(p11.y - p21.y, 2)) < CAR_LENGTH * 1.1;
  colides |= std::sqrt(std::pow(p11.x - p22.x, 2) + std::pow(p11.y - p22.y, 2)) < CAR_LENGTH * 1.1;
  colides |= std::sqrt(std::pow(p12.x - p21.x, 2) + std::pow(p12.y - p21.y, 2)) < CAR_LENGTH * 1.1;
  colides |= std::sqrt(std::pow(p12.x - p22.x, 2) + std::pow(p12.y - p22.y, 2)) < CAR_LENGTH * 1.1;

  return colides;
}

bool carConflict(sf::Vector2f carPos, sf::Angle carAngle, sf::Vector2f confPos, sf::Angle confAngle) {
  sf::Vector2f diff = carPos - confPos;
  return std::sqrt(diff.x * diff.x + diff.y * diff.y) < CAR_LENGTH * 1.1;

  sf::Vector2f p11 = carPos + sf::Vector2f(CAR_LENGTH / 2.0f * cos(carAngle.asRadians()),
                                           CAR_LENGTH / 2.0f * sin(carAngle.asRadians()));
  sf::Vector2f p12 = carPos - sf::Vector2f(CAR_LENGTH / 2.0f * cos(carAngle.asRadians()),
                                           CAR_LENGTH / 2.0f * sin(carAngle.asRadians()));
  sf::Vector2f p21 = confPos + sf::Vector2f(CAR_LENGTH / 2.0f * cos(confAngle.asRadians()),
                                            CAR_LENGTH / 2.0f * sin(confAngle.asRadians()));
  sf::Vector2f p22 = confPos - sf::Vector2f(CAR_LENGTH / 2.0f * cos(confAngle.asRadians()),
                                            CAR_LENGTH / 2.0f * sin(confAngle.asRadians()));

  bool colides = false;
  colides |= std::sqrt(std::pow(p11.x - p21.x, 2) + std::pow(p11.y - p21.y, 2)) < CAR_LENGTH * 1.1;
  colides |= std::sqrt(std::pow(p11.x - p22.x, 2) + std::pow(p11.y - p22.y, 2)) < CAR_LENGTH * 1.1;
  colides |= std::sqrt(std::pow(p12.x - p21.x, 2) + std::pow(p12.y - p21.y, 2)) < CAR_LENGTH * 1.1;
  colides |= std::sqrt(std::pow(p12.x - p22.x, 2) + std::pow(p12.y - p22.y, 2)) < CAR_LENGTH * 1.1;

  return colides;
}
