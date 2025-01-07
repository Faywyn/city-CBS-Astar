#pragma once

#include "cityGraph.h"

typedef struct _aStarNode {
  CityGraph::point point;
  double speed;

  std::pair<CityGraph::point, CityGraph::neighbor> arcFrom;

  bool operator==(const _aStarNode &other) const {
    double s = std::round(speed / SPEED_RESOLUTION);
    double oS = std::round(other.speed / SPEED_RESOLUTION);

    return point == other.point && s == oS && arcFrom.first == other.arcFrom.first &&
           arcFrom.second == other.arcFrom.second;
  }
} _aStarNode;

typedef struct _aStarConflict {
  CityGraph::point point;
  double time;

  bool operator==(const _aStarConflict &other) const {
    double t = std::round(time / TIME_RESOLUTION);
    double oT = std::round(other.time / TIME_RESOLUTION);

    return point == other.point && t == oT;
  }
} _aStarConflict;

namespace std {
template <> struct hash<_aStarNode> {
  std::size_t operator()(const _aStarNode &point) const {
    double s = std::round(point.speed / SPEED_RESOLUTION);

    return std::hash<CityGraph::point>()(point.point) ^ std::hash<double>()(s) ^
           std::hash<CityGraph::point>()(point.arcFrom.first) ^ std::hash<CityGraph::neighbor>()(point.arcFrom.second);
  }
};
template <> struct hash<_aStarConflict> {
  std::size_t operator()(const _aStarConflict &conflict) const {
    double t = std::round(conflict.time / TIME_RESOLUTION);

    return std::hash<CityGraph::point>()(conflict.point) ^ std::hash<double>()(t);
  }
};
} // namespace std

class AStar {
public:
  using node = _aStarNode;
  using conflict = _aStarConflict;

  AStar(CityGraph::point start, CityGraph::point end, const CityGraph &cityGraph);

  std::vector<node> findPath() {
    if (!processed)
      process();
    return path;
  }

private:
  bool processed = false;
  node start;
  node end;
  std::vector<node> path;
  CityGraph graph;

  void process();
};

class TimedAStar {
public:
  TimedAStar(CityGraph::point start, CityGraph::point end, const CityGraph &cityGraph,
             const std::vector<AStar::conflict> conflicts = {});

  std::vector<AStar::node> findPath() {
    if (!processed)
      process();
    return path;
  }

private:
  bool processed = false;
  AStar::node start;
  AStar::node end;
  std::vector<AStar::node> path;
  std::vector<AStar::conflict> conflicts;
  CityGraph graph;

  void process();
};
