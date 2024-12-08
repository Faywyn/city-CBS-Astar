#pragma once

#include "cityGraph.h"

typedef struct _aStarNode {
  sf::Vector2f position;
  float angle;
  float speed;

  bool operator==(const _aStarNode &other) const {
    return position == other.position && angle == other.angle && std::abs(speed - other.speed) < 0.5;
  }
} _aStarNode;

typedef struct _aStarConflict {
  sf::Vector2f position;
  float time;
  int car;

  bool operator==(const _aStarConflict &other) const {
    return std::pow(position.x - other.position.x, 2) + std::pow(position.y - other.position.y, 2) < 0.1 &&
           std::abs(time - other.time) < 0.1;
  }
} _aStarConflict;

namespace std {
template <> struct hash<_aStarNode> {
  std::size_t operator()(const _aStarNode &point) const {
    float x = point.position.x;
    float y = point.position.y;
    float angle = point.angle;

    return std::hash<float>()(x) ^ std::hash<float>()(y) ^ std::hash<float>()(angle);
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

  AStar(CityGraph::point start, CityGraph::point end, const CityGraph &cityGraph);

  std::vector<node> findPath(std::unordered_set<conflict> conflicts = {}) {
    if (!processed)
      process(conflicts);
    return path;
  }

private:
  bool processed = false;
  node start;
  node end;
  std::vector<node> path;
  CityGraph graph;

  void process(std::unordered_set<conflict> conflicts = {});
};
