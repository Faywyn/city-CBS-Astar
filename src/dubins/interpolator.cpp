#include "aStar.h"
#include "dubins.h"
#include <ompl/base/State.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <spdlog/spdlog.h>

namespace ob = ompl::base;

void DubinsInterpolator::init(CityGraph::point start_, CityGraph::point end_, double radius_) {
  startPoint = start_;
  endPoint = end_;
  radius = radius_;

  ob::DubinsStateSpace space = ob::DubinsStateSpace(radius, true);

  ob::State *start = space.allocState();
  ob::State *end = space.allocState();

  start->as<ob::DubinsStateSpace::StateType>()->setXY(startPoint.position.x, startPoint.position.y);
  start->as<ob::DubinsStateSpace::StateType>()->setYaw(startPoint.angle.asRadians());

  end->as<ob::DubinsStateSpace::StateType>()->setXY(endPoint.position.x, endPoint.position.y);
  end->as<ob::DubinsStateSpace::StateType>()->setYaw(endPoint.angle.asRadians());

  distance = space.distance(start, end);

  sf::Vector2 diff = startPoint.position - endPoint.position;
  double absDist = std::sqrt(std::pow(diff.x, 2) + std::pow(diff.y, 2));
  if (distance > absDist + 2 * M_PI * radius) {
    spdlog::warn("Distance is way too big in DubinsInterpolator");
    distance = absDist;
  }
  if (distance + 0.1 < absDist) { // 0.1 is a small tolerance
    spdlog::warn("Distance is way too small in DubinsInterpolator");
    distance = absDist;
  }

  double dx = DUBINS_INTERPOLATION_STEP / distance;
  interpolatedCurve.clear();
  interpolatedCurve.push_back(startPoint);

  for (double x = dx; x < 1; x += dx) {
    if (x == 1)
      continue;

    ob::State *state = space.allocState();
    space.interpolate(start, end, x, state);
    double x_ = state->as<ob::DubinsStateSpace::StateType>()->getX();
    double y_ = state->as<ob::DubinsStateSpace::StateType>()->getY();
    double yaw_ = state->as<ob::DubinsStateSpace::StateType>()->getYaw();

    CityGraph::point point;
    point.position = {(float)x_, (float)y_};
    point.angle = sf::radians(yaw_);

    interpolatedCurve.push_back(point);

    space.freeState(state);
  }

  interpolatedCurve.push_back(endPoint);

  numInterpolatedPoints = interpolatedCurve.size();

  space.freeState(start);
  space.freeState(end);
}

CityGraph::point DubinsInterpolator::get(double time, double startSpeed, double endSpeed) {
  double acc = (std::pow(endSpeed, 2) - std::pow(startSpeed, 2)) / (2 * distance);
  auto xFun = [&](double t) { return (0.5 * acc * std::pow(t, 2) + startSpeed * t) / distance; };

  int index = std::round((numInterpolatedPoints - 1) * xFun(time));
  index = std::clamp(index, 0, numInterpolatedPoints - 1);

  return interpolatedCurve[index];
}
