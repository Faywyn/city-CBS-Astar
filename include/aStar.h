#pragma once

#include "cityGraph.h"

class AStar {
public:
  typedef struct node {
    sf::Vector2f position;
    float angle;
    float speed;

    bool operator==(const node &other) const {
      return position == other.position && angle == other.angle && std::abs(speed - other.speed) < 0.1;
    }
  } node;

  AStar(graphPoint start, graphPoint end, const CityGraph &cityGraph);

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

namespace std {
template <> struct hash<AStar::node> {
  std::size_t operator()(const AStar::node &point) const {
    float x = point.position.x;
    float y = point.position.y;
    float angle = point.angle;

    return std::hash<float>()(x) ^ std::hash<float>()(y) ^ std::hash<float>()(angle);
  }
};
} // namespace std
