/**
 * @file interpolator.cpp
 * @brief Implementation of Dubins path interpolation
 * 
 * This file implements the DubinsInterpolator class which uses OMPL's Dubins curves
 * to compute smooth paths between two poses (position + orientation). Dubins curves
 * are the shortest paths for a vehicle with a minimum turning radius constraint.
 */
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

  // Create a Dubins state space with the given turning radius
  // The second parameter (true) indicates symmetric Dubins paths
  ob::DubinsStateSpace space = ob::DubinsStateSpace(radius, true);

  // Allocate OMPL states for start and end poses
  ob::State *start = space.allocState();
  ob::State *end = space.allocState();

  // Set start and end poses (position + orientation)
  start->as<ob::DubinsStateSpace::StateType>()->setXY(startPoint.position.x, startPoint.position.y);
  start->as<ob::DubinsStateSpace::StateType>()->setYaw(startPoint.angle.asRadians());

  end->as<ob::DubinsStateSpace::StateType>()->setXY(endPoint.position.x, endPoint.position.y);
  end->as<ob::DubinsStateSpace::StateType>()->setYaw(endPoint.angle.asRadians());

  // Compute the Dubins path distance
  distance = space.distance(start, end);

  // Validate the computed distance against straight-line distance
  sf::Vector2 diff = startPoint.position - endPoint.position;
  double absDist = std::sqrt(std::pow(diff.x, 2) + std::pow(diff.y, 2));
  
  // Distance should be at most straight-line distance plus maximum arc length
  if (distance > absDist + 2 * M_PI * radius) {
    spdlog::warn("Distance is way too big in DubinsInterpolator");
    distance = absDist;
  }
  
  // Distance should be at least the straight-line distance (with small tolerance)
  constexpr double DISTANCE_TOLERANCE = 0.1;
  if (distance + DISTANCE_TOLERANCE < absDist) {
    spdlog::warn("Distance is way too small in DubinsInterpolator");
    distance = absDist;
  }

  // Compute interpolation step size in [0,1] parameter space
  double dx = DUBINS_INTERPOLATION_STEP / distance;
  interpolatedCurve.clear();
  interpolatedCurve.push_back(startPoint);

  // Interpolate points along the Dubins curve
  for (double x = dx; x < 1; x += dx) {
    if (x == 1)  // Skip endpoint to avoid duplication
      continue;

    ob::State *state = space.allocState();
    space.interpolate(start, end, x, state);
    
    // Extract pose from interpolated state
    double x_ = state->as<ob::DubinsStateSpace::StateType>()->getX();
    double y_ = state->as<ob::DubinsStateSpace::StateType>()->getY();
    double yaw_ = state->as<ob::DubinsStateSpace::StateType>()->getYaw();

    CityGraph::point point;
    point.position = {(float)x_, (float)y_};
    point.angle = sf::radians(yaw_);

    interpolatedCurve.push_back(point);

    space.freeState(state);
  }

  // Add endpoint explicitly
  interpolatedCurve.push_back(endPoint);

  numInterpolatedPoints = interpolatedCurve.size();

  space.freeState(start);
  space.freeState(end);
}

CityGraph::point DubinsInterpolator::get(double time, double startSpeed, double endSpeed) {
  // Calculate acceleration based on start/end speeds and path distance
  // Using kinematic equation: v^2 = u^2 + 2as
  double acc = (std::pow(endSpeed, 2) - std::pow(startSpeed, 2)) / (2 * distance);
  
  // Define position function using kinematic equation: s = ut + 0.5at^2
  // Normalized to [0,1] by dividing by total distance
  auto xFun = [&](double t) { return (0.5 * acc * std::pow(t, 2) + startSpeed * t) / distance; };

  // Map normalized position to interpolated curve index
  int index = std::round((numInterpolatedPoints - 1) * xFun(time));
  index = std::clamp(index, 0, numInterpolatedPoints - 1);

  return interpolatedCurve[index];
}
