#pragma once

#include "aStar.h"
#include "cityGraph.h"

#include <ompl/base/State.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/DubinsStateSpace.h>

namespace ob = ompl::base;

class Dubins {
public:
  Dubins(CityGraph::point start, CityGraph::neighbor end);                    // Running at max speed
  Dubins(CityGraph::point start, CityGraph::neighbor end, double startSpeed); // Accelerating to max speed
  Dubins(CityGraph::point start, CityGraph::neighbor end, double startSpeed, double endSpeed);
  ~Dubins();

  double distance() { return endPoint.distance; }
  double time();
  CityGraph::point point(double time);
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

class DubinsPath {
public:
  DubinsPath(std::vector<AStar::node> path);

  std::vector<CityGraph::point> path();

private:
  void process();

  std::vector<AStar::node> path_;
  std::vector<CityGraph::point> pathProcessed_;
};
