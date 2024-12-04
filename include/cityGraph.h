#pragma once
#include <unordered_set>

#include "cityMap.h"
#include "config.h"

typedef struct graphPoint {
  sf::Vector2f position;
  float angle;

  bool operator==(const graphPoint &other) const {
    float x = std::round(position.x / CELL_SIZE) * CELL_SIZE;
    float y = std::round(position.y / CELL_SIZE) * CELL_SIZE;
    float oX = std::round(other.position.x / CELL_SIZE) * CELL_SIZE;
    float oY = std::round(other.position.y / CELL_SIZE) * CELL_SIZE;

    return x == oX && y == oY && angle == other.angle;
  }
} graphPoint;

typedef struct neighbor {
  graphPoint point;
  float maxSpeed;
  float distance;
} neighbor;

// Hash (rounded)
namespace std {
template <> struct hash<graphPoint> {
  std::size_t operator()(const graphPoint &point) const {
    float x = std::round(point.position.x / CELL_SIZE) * CELL_SIZE;
    float y = std::round(point.position.y / CELL_SIZE) * CELL_SIZE;

    return std::hash<float>()(x) ^ std::hash<float>()(y) ^ std::hash<float>()(point.angle);
  }
};
} // namespace std

class CityGraph {
public:
  void createGraph(const CityMap &cityMap);

  // Getters
  std::unordered_map<graphPoint, std::vector<neighbor>> getNeighbors() const { return neighbors; }
  std::unordered_set<graphPoint> getGraphPoints() const { return graphPoints; }
  graphPoint getRandomPoint() const;

private:
  std::unordered_map<graphPoint, std::vector<neighbor>> neighbors;
  std::unordered_set<graphPoint> graphPoints;

  void linkPoints(const graphPoint &point, const graphPoint &neighbor);
  bool canLink(const graphPoint &point1, const graphPoint &point2, float speed, float *distance) const;
};
