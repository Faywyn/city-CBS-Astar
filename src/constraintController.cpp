#include <iostream>
#include <spdlog/spdlog.h>

#include "aStar.h"
#include "dubins.h"

void ConstraintController::addConstraint(AStar::conflict constraint) {
  if (constraint.car < 0) {
    spdlog::error("Invalid car index for constraint");
    throw std::runtime_error("Invalid car index for constraint");
  }

  while (constraints.size() <= constraint.car) {
    constraints.push_back(std::vector<std::vector<AStar::conflict>>());
  }

  while (constraints[constraint.car].size() <= constraint.time) {
    constraints[constraint.car].push_back(std::vector<AStar::conflict>());
  }

  constraints[constraint.car][constraint.time].push_back(constraint);
}

bool ConstraintController::hasConstraint(AStar::conflict constraint) {
  if (constraint.car < 0) {
    spdlog::error("Invalid car index for constraint");
    throw std::runtime_error("Invalid car index for constraint");
  }
  if (constraints.size() <= constraint.car) {
    return false;
  }
  for (int t = std::max(0, constraint.time - 1);
       t < std::min((int)constraints[constraint.car].size(), constraint.time + 1); t++) {
    for (auto c : constraints[constraint.car][t]) {
      if (c == constraint) {
        return true;
      }
    }
  }

  return false;
}

ConstraintController ConstraintController::copy() {
  ConstraintController cc;

  cc.constraints = constraints;

  return cc;
}

ConstraintController ConstraintController::copy(std::vector<int> cars) {
  ConstraintController cc;

  for (int car : cars) {
    if (constraints.size() > car) {
      cc.constraints.push_back(constraints[car]);
    } else {
      cc.constraints.push_back(std::vector<std::vector<AStar::conflict>>());
    }
  }

  return cc;
}

bool ConstraintController::checkConstraints(int car, double speed, double newSpeed, double time, CityGraph::point from,
                                            CityGraph::neighbor to) {

  Dubins dubin = Dubins(from, to, speed, newSpeed);
  float duration = 2 * to.distance / (speed + newSpeed);

  int tMin = std::round(time / SIM_STEP_TIME);
  tMin = tMin < 0 ? 0 : tMin;
  int tMax = std::round((time + duration) / SIM_STEP_TIME);
  if (constraints.size() > car && constraints[car].size() < tMax) {
    tMax = constraints[car].size();
  }

  if (constraints.size() <= car || constraints[car].size() <= tMin) {
    return false;
  }

  double distance = to.distance;
  double acc = (std::pow(newSpeed, 2) - std::pow(speed, 2)) / (2 * distance);
  auto xFun = [&](double t) { return (0.5 * acc * t * t + speed * t) / distance; };

  for (int t = tMin; t < tMax; t += 1) {
    bool precise = false;
    sf::Vector2f pos = from.position + (to.point.position - from.position) * (float)xFun(t * SIM_STEP_TIME - time);
    CityGraph::point point;
    if (constraints[car].size() <= t)
      continue;

    for (auto c : constraints[car][t]) {
      if (precise) {
        if (carConflict(point.position, point.angle, c.point.position, c.point.angle)) {
          return true;
        }
      } else {
        sf::Vector2f diff = pos - c.point.position;
        double dist = std::sqrt(diff.x * diff.x + diff.y * diff.y);
        if (dist < 2 * CAR_LENGTH) {
          precise = true;
          point = dubin.point(t * SIM_STEP_TIME - time);
          if (carConflict(point.position, point.angle, c.point.position, c.point.angle)) {
            return true;
          }
        }
      }
    }
  }

  return false;
}
