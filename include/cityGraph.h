#pragma once
#include <unordered_set>

#include "cityMap.h"
#include "config.h"

typedef struct _cityGraphPoint {
  sf::Vector2f position;
  float angle;

  bool operator==(const _cityGraphPoint &other) const {
    float x = std::round(position.x / CELL_SIZE) * CELL_SIZE;
    float y = std::round(position.y / CELL_SIZE) * CELL_SIZE;
    float oX = std::round(other.position.x / CELL_SIZE) * CELL_SIZE;
    float oY = std::round(other.position.y / CELL_SIZE) * CELL_SIZE;

    return x == oX && y == oY && angle == other.angle;
  }
} _cityGraphPoint;

typedef struct _cityGraphNeighbor {
  _cityGraphPoint point;
  float maxSpeed;
  float turningRadius;
  float distance;
} _cityGraphNeighbor;

// Hash (rounded)
namespace std {
template <> struct hash<_cityGraphPoint> {
  std::size_t operator()(const _cityGraphPoint &point) const {
    float x = std::round(point.position.x / CELL_SIZE) * CELL_SIZE;
    float y = std::round(point.position.y / CELL_SIZE) * CELL_SIZE;

    return std::hash<float>()(x) ^ std::hash<float>()(y) ^ std::hash<float>()(point.angle);
  }
};
} // namespace std

class CityGraph {
public:
  using point = _cityGraphPoint;
  using neighbor = _cityGraphNeighbor;

  void createGraph(const CityMap &cityMap);

  // Getters
  std::unordered_map<point, std::vector<neighbor>> getNeighbors() const { return neighbors; }
  std::unordered_set<point> getGraphPoints() const { return graphPoints; }
  point getRandomPoint() const;

private:
  std::unordered_map<point, std::vector<neighbor>> neighbors;
  std::unordered_set<point> graphPoints;

  void linkPoints(const point &point1, const point &point2);
  bool canLink(const point &point1, const point &point2, float speed, float *distance) const;
};
