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
  /**
   * @brief Initialize the Dubins path with start and end points and a radius
   * @param start The start point
   * @param end The end point
   * @param radius The turning radius
   */
  void init(_cityGraphPoint start, _cityGraphPoint end, double radius);

  /**
   * @brief Get the position at a certain time
   * @param time The time
   * @param startSpeed The speed at the start point
   * @param endSpeed The speed at the end point
   * @return The position at the time
   */
  _cityGraphPoint get(double time, double startSpeed, double endSpeed);

  /**
   * @brief Get the duration of the Dubins path based on the start and end speeds
   * @param startSpeed The speed at the start point
   * @param endSpeed The speed at the end point
   * @return The duration of the Dubins path
   */
  double getDuration(double startSpeed, double endSpeed) { return 2 * distance / (startSpeed + endSpeed); }

  /**
   * @brief Get the distance between the start and end points depending on the dubins path
   * @return The distance
   */
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
