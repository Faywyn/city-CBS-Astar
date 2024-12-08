#pragma once

#include "aStar.h"
#include "cityGraph.h"

#include <ompl/base/State.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/DubinsStateSpace.h>

namespace ob = ompl::base;

class Dubins {
public:
  Dubins(AStar::node start, AStar::node end);
  ~Dubins();

  float distance();
  float time();
  CityGraph::point point(float time);
  std::vector<CityGraph::point> path();

private:
  ob::DubinsStateSpace *space;
  ob::State *start;
  ob::State *end;

  AStar::node startNode;
  AStar::node endNode;
  float avgSpeed;
  float radius;
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
