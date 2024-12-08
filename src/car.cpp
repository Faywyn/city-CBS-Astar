#include "car.h"
#include "config.h"
#include "dubins.h"
#include "utils.h"

#include <iostream>

void Car::move() {
  if (currentPoint >= (int)path.size())
    return;

  currentPoint++;
}

void Car::render(sf::RenderWindow &window) {
  if (1 + currentPoint >= (int)path.size())
    return;

  sf::Vector2f point = path[currentPoint];
  sf::Vector2f nextPoint = path[currentPoint + 1];

  sf::RectangleShape shape(sf::Vector2f(CAR_LENGTH, CAR_WIDTH));
  shape.setOrigin(CAR_LENGTH / 2.0f, CAR_WIDTH / 2.0f);
  shape.setPosition(point);
  shape.setRotation(atan2(nextPoint.y - point.y, nextPoint.x - point.x) * 180.0f / M_PI);
  if (debug)
    shape.setFillColor(sf::Color(255, 0, 0));
  else
    shape.setFillColor(sf::Color(80, 112, 156));
  window.draw(shape);

  if (!debug)
    return;

  // Render speed, elapsed time, remaining time, and distance
  int speed = (int)(getSpeed() * 3.6f);
  int dSpeed = (getSpeed() * 3.6f - (float)speed) * 100;
  sf::Font font = loadFont();
  sf::Text text;
  text.setFont(font);
  text.setCharacterSize(24);
  text.setFillColor(sf::Color::White);
  text.setPosition(getPosition());
  text.setString(std::to_string(speed) + "." + std::to_string(dSpeed) + " km/h" + "\n" +
                 std::to_string((int)getElapsedTime()) + "s / " + std::to_string((int)getRemainingTime()) + "s" + "\n" +
                 std::to_string((int)getElapsedDistance()) + "m / " + std::to_string((int)getRemainingDistance()) +
                 "m");
  text.setOutlineColor(sf::Color::Black);
  text.setOutlineThickness(1.0f);
  text.scale(0.1f, 0.1f);
  text.setOrigin(text.getLocalBounds().width / 2.0f, text.getLocalBounds().height / 2.0f);
  window.draw(text);

  // Render path
  for (int i = currentPoint; i < (int)path.size() - 1; i++) {
    sf::Vertex line[] = {sf::Vertex(path[i]), sf::Vertex(path[i + 1])};
    line[0].color = sf::Color(255, 255, 255);
    line[1].color = sf::Color(255, 255, 255);
    window.draw(line, 2, sf::Lines);
  }
}

void Car::assignPath(std::vector<AStar::node> path) {
  this->path.clear();
  DubinsPath dubins(path);
  std::vector<CityGraph::point> dubinsPath_ = dubins.path();
  for (CityGraph::point point : dubinsPath_) {
    this->path.push_back(point.position);
  }
}

float Car::getSpeed() {
  if (currentPoint >= (int)path.size() - 1)
    return 0;

  sf::Vector2f diff = path[currentPoint + 1] - path[currentPoint];
  return sqrt(diff.x * diff.x + diff.y * diff.y) / SIM_STEP_TIME;
}

float Car::getRemainingDistance() {
  if (currentPoint >= (int)path.size() - 1)
    return 0;

  float dist = 0;
  for (int i = currentPoint; i < (int)path.size() - 1; i++) {
    sf::Vector2f diff = path[i + 1] - path[i];
    dist += sqrt(diff.x * diff.x + diff.y * diff.y);
  }

  return dist;
}

float Car::getElapsedDistance() {
  float dist = 0;
  for (int i = 0; i < currentPoint; i++) {
    sf::Vector2f diff = path[i + 1] - path[i];
    dist += sqrt(diff.x * diff.x + diff.y * diff.y);
  }

  return dist;
}

float Car::getRemainingTime(bool fromStart) { return (path.size() - (fromStart ? 0 : currentPoint)) * SIM_STEP_TIME; }
float Car::getElapsedTime() { return currentPoint * SIM_STEP_TIME; }
