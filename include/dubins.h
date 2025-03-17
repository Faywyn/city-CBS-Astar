/**
 * @file dubins.h
 * @brief Dubins path
 *
 * This file contains the Dubins class. It is used to calculate the path between two points in the city graph.
 * It will be used to render cars in the city and check for collisions.
 */
#pragma once

#include "aStar.h"
#include "cityGraph.h"

#include <ompl/base/State.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/DubinsStateSpace.h>

namespace ob = ompl::base;

/**
 * @class Dubins
 * @brief Dubins path used to calculate the path between two points in the city graph.
 *
 * This class represents a Dubins path used to calculate the path between two points in the city graph.
 * Given the start and end points, it calculates the path, the distance and the time to reach the end point.
 */
class Dubins {
public:
  /**
   * @brief Constructor with start and end points.
   *
   * The class will be initialized with the start and end points. The car will run without speed limits.
   *
   * @param start The start point
   * @param end The end point
   */
  Dubins(CityGraph::point start, CityGraph::neighbor end);

  /**
   * @brief Constructor with start point, end point and start speed.
   *
   * The class will be initialized with the start and end points and the start speed. The car will accelerate to the
   * maximum speed.
   *
   * @param start The start point
   * @param end The end point
   * @param startSpeed The start speed
   */
  Dubins(CityGraph::point start, CityGraph::neighbor end, double startSpeed);

  /**
   * @brief Constructor with start point, end point, start speed and end speed.
   *
   * The class will be initialized with the start and end points, the start and end speeds. The car will accelerate
   * uniformly to the maximum speed.
   *
   * @param start The start point
   * @param end The end point
   * @param startSpeed The start speed
   * @param endSpeed The end speed
   */
  Dubins(CityGraph::point start, CityGraph::neighbor end, double startSpeed, double endSpeed);

  /**
   * @brief Destructor
   */
  ~Dubins();

  /**
   * @brief Get the distance to reach the end point
   * @return The distance
   */
  double distance() { return endPoint.distance; }

  /**
   * @brief Get the time to reach the end point
   * @return The time
   */
  double time();

  /**
   * @brief Get the point at a certain time in the path using interpolation
   * @param time The time
   * @return The point
   */
  CityGraph::point point(double time);

  /**
   * @brief Get the path using interpolation
   * @return The path
   */
  std::vector<CityGraph::point> path();

private:
  ob::DubinsStateSpace *space;
  ob::State *start;
  ob::State *end;

  CityGraph::point startPoint;
  CityGraph::neighbor endPoint;
  double startSpeed;
  double endSpeed;
  double avgSpeed;
};

/**
 * @class DubinsPath
 * @brief Dubins path used to calculate the path between two points in the city graph.
 *
 * This class represents a Dubins path used to calculate the path between two points in the city graph.
 * Given the start and end points, it calculates the path, the distance and the time to reach the end point.
 */
class DubinsPath {
public:
  /**
   * @brief Constructor with path.
   *
   * The class will be initialized with the path.
   *
   * @param path The path
   */
  DubinsPath(std::vector<AStar::node> path);

  /**
   * @brief Get the path
   */
  std::vector<CityGraph::point> path();

private:
  void process();

  std::vector<AStar::node> path_;
  std::vector<CityGraph::point> pathProcessed_;
};
