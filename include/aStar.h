#pragma once

#include "cityGraph.h"

typedef struct _aStarNode {
  CityGraph::point point;
  float speed;

  std::pair<CityGraph::point, CityGraph::neighbor> arcFrom;
  bool start = false;

  bool operator==(const _aStarNode &other) const { return point == other.point && std::abs(speed - other.speed) < 0.1; }
} _aStarNode;

typedef struct _aStarConflict {
  sf::Vector2f position;
  float time;

  bool operator==(const _aStarConflict &other) const {
    return std::pow(position.x - other.position.x, 2) + std::pow(position.y - other.position.y, 2) < 0.1 &&
           std::abs(time - other.time) < 0.1;
  }
} _aStarConflict;

namespace std {
template <> struct hash<_aStarNode> {
  std::size_t operator()(const _aStarNode &point) const {
    return std::hash<CityGraph::point>()(point.point) ^ std::hash<float>()(point.speed);
  }
};
template <> struct hash<_aStarConflict> {
  std::size_t operator()(const _aStarConflict &conflict) const {
    float x = (int)conflict.position.x;
    float y = (int)conflict.position.y;
    float time = (int)conflict.time;

    return std::hash<float>()(x) ^ std::hash<float>()(y) ^ std::hash<float>()(time);
  }
};
} // namespace std

class AStar {
public:
  using node = _aStarNode;
  using conflict = _aStarConflict;

  AStar(CityGraph::point start, CityGraph::point end, const CityGraph &cityGraph,
        const std::vector<conflict> conflicts = {});

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
  std::vector<conflict> conflicts;
  CityGraph graph;

  void process();
};
