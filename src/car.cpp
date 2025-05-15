/**
 * @file car.cpp
 * @brief Car class implementation
 *
 * This file contains the implementation of the Car class.
 */
#include "car.h"
#include "SFML/Graphics/Text.hpp"
#include "SFML/System/Angle.hpp"
#include "config.h"
#include "dubins.h"
#include "utils.h"

#include <iostream>
#include <random>

Car::Car() {
  std::vector<sf::Color> colors = {sf::Color(50, 120, 190), sf::Color(183, 132, 144), sf::Color(105, 101, 89),
                                   sf::Color(182, 18, 34),  sf::Color(24, 25, 24),    sf::Color(17, 86, 122)};
  color = colors[rand() % colors.size()];
}

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
  shape.setOrigin({CAR_LENGTH / 2.0f, CAR_WIDTH / 2.0f});
  shape.setPosition(point);
  shape.setRotation(sf::radians(atan2(nextPoint.y - point.y, nextPoint.x - point.x)));
  if (debug)
    shape.setFillColor(sf::Color(255, 0, 0));
  else
    shape.setFillColor(color);
  window.draw(shape);

  if (!debug)
    return;

  // Render speed, elapsed time, remaining time, and distance
  int speed = (int)(getSpeed() * 3.6f);
  int dSpeed = (getSpeed() * 3.6f - (double)speed) * 100;
  sf::Font font = loadFont();
  sf::Text text(font);
  text.setCharacterSize(24);
  text.setFillColor(sf::Color::White);
  text.setPosition(getPosition());
  text.setString(std::to_string(speed) + "." + std::to_string(dSpeed) + " km/h" + "\n" +
                 std::to_string((int)getElapsedTime()) + "s / " + std::to_string((int)getRemainingTime()) + "s" + "\n" +
                 std::to_string((int)getElapsedDistance()) + "m / " + std::to_string((int)getRemainingDistance()) +
                 "m");
  text.setOutlineColor(sf::Color::Black);
  text.setOutlineThickness(1.0f);
  text.scale({0.1f, 0.1f});
  text.setOrigin({text.getLocalBounds().position.x / 2.0f, text.getLocalBounds().position.y / 2.0f});
  window.draw(text);

  // Render path
  for (int i = currentPoint; i < (int)path.size() - 1; i++) {
    sf::Vertex line[] = {{path[i]}, {path[i + 1]}};
    line[0].color = sf::Color(255, 255, 255);
    line[1].color = sf::Color(255, 255, 255);
    window.draw(line, 2, sf::PrimitiveType::Lines);
  }
}

void Car::assignPath(std::vector<AStar::node> path) {
  this->path.clear();
  this->aStarPath = path;
  DubinsPath dubins(path);
  std::vector<CityGraph::point> dubinsPath_ = dubins.path();
  for (CityGraph::point point : dubinsPath_) {
    this->path.push_back(point.position);
  }
  currentPoint = 0;
}

void Car::assignExistingPath(std::vector<sf::Vector2f> path) {
  this->path = path;
  currentPoint = 0;
}

double Car::getSpeed() {
  if (currentPoint >= (int)path.size() - 1)
    return 0;

  sf::Vector2f diff = path[currentPoint + 1] - path[currentPoint];
  return sqrt(diff.x * diff.x + diff.y * diff.y) / SIM_STEP_TIME;
}

double Car::getSpeedAt(int index) {
  if (index >= (int)path.size() - 1)
    return 0;

  sf::Vector2f diff = path[index + 1] - path[index];
  return sqrt(diff.x * diff.x + diff.y * diff.y) / SIM_STEP_TIME;
}

double Car::getRemainingTime() { return (double)(path.size() - currentPoint) * SIM_STEP_TIME; }
double Car::getElapsedTime() { return (double)currentPoint * SIM_STEP_TIME; }
double Car::getPathTime() { return (double)path.size() * SIM_STEP_TIME; }

double Car::getRemainingDistance() {
  double dist = 0;
  for (int i = currentPoint; i < (int)path.size() - 1; i++) {
    sf::Vector2f diff = path[i + 1] - path[i];
    dist += sqrt(diff.x * diff.x + diff.y * diff.y);
  }

  return dist;
}

double Car::getElapsedDistance() {
  double dist = 0;
  for (int i = 0; i < currentPoint; i++) {
    sf::Vector2f diff = path[i + 1] - path[i];
    dist += sqrt(diff.x * diff.x + diff.y * diff.y);
  }

  return dist;
}

double Car::getPathLength() {
  double dist = 0;
  for (int i = 0; i < (int)path.size() - 1; i++) {
    sf::Vector2f diff = path[i + 1] - path[i];
    dist += sqrt(diff.x * diff.x + diff.y * diff.y);
  }

  return dist;
}

void Car::chooseRandomStartEndPath(CityGraph &graph, CityMap &cityMap) {
  CityGraph::point start;
  CityGraph::point end;

  double minDistance = std::max(graph.getWidth(), graph.getHeight()) / 2.0;
  std::vector<AStar::node> path;

  do {
    path.clear();
    start = graph.getRandomPoint();
    end = graph.getRandomPoint();

    if (std::sqrt(std::pow(start.position.x - end.position.x, 2) + std::pow(start.position.y - end.position.y, 2)) <
        minDistance)
      continue;

    AStar aStar(start, end, graph);
    path = aStar.findPath();

    if (!path.empty() && (int)path.size() >= 3) {
      TimedAStar timedAStar(start, end, graph, nullptr, 0);
      path.clear();
      path = timedAStar.findPath();
    }
  } while (path.empty() || (int)path.size() < 3);

  this->assignStartEnd(start, end);
  this->assignPath(path);
}

double Car::getAverageSpeed(CityGraph &graph) {
  double dist = 0;
  double time = 0;
  auto outOfBounds = [&](sf::Vector2f p) {
    return p.x < 0 || p.y < 0 || p.x > graph.getWidth() || p.y > graph.getWidth();
  };

  for (int i = 0; i < (int)path.size() - 1; i++) {
    if (outOfBounds(path[i]) || outOfBounds(path[i + 1]))
      continue;

    sf::Vector2f diff = path[i + 1] - path[i];
    dist += sqrt(diff.x * diff.x + diff.y * diff.y);
    time += SIM_STEP_TIME;
  }

  if (time == 0)
    return 0;

  return dist / time;
}
