#include "aStar.h"
#include "dubins.h"

#include <iostream>

// All constraints are added in the range [time - CBS_PRECISION_FACTOR, time + CBS_PRECISION_FACTOR]
void ConstraintController::addConstraint(AStar::conflict constraint, bool global) {
  if (global || constraint.car == -1) {
    while (globalConstraints.size() <= constraint.time) {
      globalConstraints.push_back(std::vector<AStar::conflict>());
    }

    constraint.car = -1;
    globalConstraints[constraint.time].push_back(constraint);
  } else {
    while (constraints.size() <= constraint.car) {
      constraints.push_back(std::vector<std::vector<AStar::conflict>>());
    }

    while (constraints[constraint.car].size() <= constraint.time) {
      constraints[constraint.car].push_back(std::vector<AStar::conflict>());
    }

    constraints[constraint.car][constraint.time].push_back(constraint);
  }
}

bool ConstraintController::hasConstraint(AStar::conflict constraint, bool global) {
  if (global || constraint.car == -1) {
    constraint.car = -1;
    for (auto c : globalConstraints[constraint.time]) {
      if (c == constraint) {
        return true;
      }
    }
  } else {
    if (constraints.size() <= constraint.car) {
      return false;
    }
    for (int t = std::max(0, constraint.time - CBS_PRECISION_FACTOR);
         t < std::min((int)constraints[constraint.car].size(), constraint.time + CBS_PRECISION_FACTOR); t++) {
      for (auto c : constraints[constraint.car][t]) {
        if (c == constraint) {
          return true;
        }
      }
    }
  }

  return false;
}

ConstraintController ConstraintController::copy() {
  ConstraintController cc;

  cc.constraints = constraints;
  cc.globalConstraints = globalConstraints;

  return cc;
}

ConstraintController ConstraintController::copy(std::vector<int> cars) {
  ConstraintController cc;
  cc.globalConstraints = globalConstraints;

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

  if (globalConstraints.size() < tMin && (constraints.size() <= car || constraints[car].size() <= tMin)) {
    return false;
  }

  double distance = to.distance;
  double acc = (std::pow(newSpeed, 2) - std::pow(speed, 2)) / (2 * distance);
  auto xFun = [&](double t) { return (0.5 * acc * t * t + speed * t) / distance; };

  for (int t = tMin; t < tMax; t += CBS_PRECISION_FACTOR) {
    bool precise = false;
    sf::Vector2f pos = from.position + (to.point.position - from.position) * (float)xFun(t * SIM_STEP_TIME - time);
    CityGraph::point point;
    // = dubin.point(t * SIM_STEP_TIME - time);
    for (int _t = t; _t < t + CBS_PRECISION_FACTOR; _t++) {
      if (constraints.size() > car && constraints[car].size() > _t) {
        for (auto c : constraints[car][_t]) {
          if (precise) {
            if (carConflict(point.position, point.angle, c.point.position, c.point.angle)) {
              return true;
            }
          } else {
            sf::Vector2f diff = pos - c.point.position;
            double dist = std::sqrt(diff.x * diff.x + diff.y * diff.y);
            if (dist < 2 * CAR_LENGTH) {
              precise = true;
              point = dubin.point(_t * SIM_STEP_TIME - time);
              if (carConflict(point.position, point.angle, c.point.position, c.point.angle)) {
                return true;
              }
            }
          }
        }
      }

      if (globalConstraints.size() > _t) {
        for (auto c : globalConstraints[_t]) {
          if (precise) {

            if (carConflict(point.position, point.angle, c.point.position, c.point.angle)) {
              return true;
            }
          } else {
            sf::Vector2f diff = pos - c.point.position;
            double dist = std::sqrt(diff.x * diff.x + diff.y * diff.y);
            if (dist < 2 * CAR_LENGTH) {
              precise = true;
              point = dubin.point(_t * SIM_STEP_TIME - time);
              if (carConflict(point.position, point.angle, c.point.position, c.point.angle)) {
                return true;
              }
            }
          }
        }
      }
    }
  }

  return false;
}
