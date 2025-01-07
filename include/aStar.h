#pragma once

#include "cityGraph.h"

typedef struct _aStarNode {
  CityGraph::point point;
  double speed;

  std::pair<CityGraph::point, CityGraph::neighbor> arcFrom;
  bool notMoving = false;
  bool start = false;

  bool operator==(const _aStarNode &other) const {
    double s = std::round(speed / SPEED_RESOLUTION);
    double oS = std::round(other.speed / SPEED_RESOLUTION);

    return point == other.point && arcFrom.first == other.arcFrom.first && arcFrom.second == other.arcFrom.second &&
           s == oS;
  }
} _aStarNode;

typedef struct _aStarConflict {
  sf::Vector2f position;
  double time;

  bool operator==(const _aStarConflict &other) const {
    double t = std::round(time / TIME_RESOLUTION);
    double oT = std::round(other.time / TIME_RESOLUTION);

    return position == other.position && t == oT;
  }
} _aStarConflict;

namespace std {
template <> struct hash<_aStarNode> {
  std::size_t operator()(const _aStarNode &point) const { return std::hash<CityGraph::point>()(point.point); }
};
template <> struct hash<_aStarConflict> {
  std::size_t operator()(const _aStarConflict &conflict) const {
    double t = std::round(conflict.time / TIME_RESOLUTION);
    double x = std::round(conflict.position.x / CELL_SIZE);
    double y = std::round(conflict.position.y / CELL_SIZE);

    return std::hash<double>()(x) ^ std::hash<double>()(y) ^ std::hash<double>()(t);
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
