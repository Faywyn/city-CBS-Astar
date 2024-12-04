#include "car.h"
#include "config.h"
#include "utils.h"

#include <iostream>
#include <ompl/base/State.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/DubinsStateSpace.h>

namespace ob = ompl::base;

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

    // sf::CircleShape circle(0.5);
    // circle.setOrigin(0.5, 0.5);
    // circle.setPosition(path[i]);
    // circle.setFillColor(sf::Color(255, 255, 255));
    // window.draw(circle);
  }
}

void Car::assignPath(std::vector<AStar::node> path) {
  this->path.clear();

  for (int i = 1; i < (int)path.size(); i++) {
    AStar::node prev = path[i - 1];
    AStar::node current = path[i];

    float radius = turningRadius(prev.speed);
    auto space = ob::DubinsStateSpace(radius);
    ob::RealVectorBounds bounds(2);
    space.setBounds(bounds);

    ob::State *start = space.allocState();
    ob::State *end = space.allocState();

    start->as<ob::DubinsStateSpace::StateType>()->setXY(prev.position.x, prev.position.y);
    start->as<ob::DubinsStateSpace::StateType>()->setYaw(prev.angle);

    end->as<ob::DubinsStateSpace::StateType>()->setXY(current.position.x, current.position.y);
    end->as<ob::DubinsStateSpace::StateType>()->setYaw(current.angle);

    float distance = space.distance(start, end);
    float acc =
        current.speed == prev.speed ? 0.0f : (current.speed > prev.speed ? CAR_ACCELERATION : -CAR_ACCELERATION);
    auto x = [distance, acc, prev](float t) { return (0.5f * acc * t * t + prev.speed * t) / distance; };

    float t = 0;
    while (x(t) <= 1 && x(t) >= 0) {
      ob::State *state = space.allocState();
      space.interpolate(start, end, x(t), state);

      float x = state->as<ob::DubinsStateSpace::StateType>()->getX();
      float y = state->as<ob::DubinsStateSpace::StateType>()->getY();

      this->path.push_back({x, y});

      t += SIM_STEP_TIME;
    }
  }

  currentPoint = 0;
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

float Car::getRemainingTime() { return (path.size() - currentPoint) * SIM_STEP_TIME; }
float Car::getElapsedTime() { return currentPoint * SIM_STEP_TIME; }
