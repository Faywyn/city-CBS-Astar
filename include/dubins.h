/**
 * @file dubins.h
 * @brief Dubins path
 *
 * This file contains the Dubins class. It is used to calculate the path between two points in the city graph.
 * It will be used to render cars in the city and check for collisions.
 */
#pragma once

#include "cityGraph.h"
#include <vector>

class AStar;

class DubinsInterpolator {
public:
  void init(_cityGraphPoint start, _cityGraphPoint end, double radius);

  _cityGraphPoint get(double time, double startSpeed, double endSpeed);

  double getDuration(double startSpeed, double endSpeed) { return 2 * distance / (startSpeed + endSpeed); }
  double getDistance() { return distance; }

private:
  _cityGraphPoint startPoint;
  _cityGraphPoint endPoint;
  double distance;
  double radius;
  int numInterpolatedPoints;

  // Points spaced by DUBINS_INTERPOLATION_STEP. The first point and the last point are always the start and end points.
  std::vector<_cityGraphPoint> interpolatedCurve;
};
