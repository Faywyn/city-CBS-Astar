#pragma once
#include <unordered_set>

#include "cityMap.h"
#include "config.h"
#include "utils.h"

typedef struct _cityGraphPoint {
  sf::Vector2f position;
  double angle;

  bool operator==(const _cityGraphPoint &other) const {
    double x = std::round(position.x / CELL_SIZE);
    double y = std::round(position.y / CELL_SIZE);
    double a = std::round(normalizeAngle(angle) / ANGLE_RESOLUTION);
    double oX = std::round(other.position.x / CELL_SIZE);
    double oY = std::round(other.position.y / CELL_SIZE);
    double oA = std::round(normalizeAngle(other.angle) / ANGLE_RESOLUTION);

    return x == oX && y == oY && a == oA;
  }
} _cityGraphPoint;

typedef struct _cityGraphNeighbor {
  _cityGraphPoint point;
  double maxSpeed;
  double turningRadius;
  double distance;

  bool operator==(const _cityGraphNeighbor &other) const {
    return point == other.point && maxSpeed == other.maxSpeed && turningRadius == other.turningRadius &&
           distance == other.distance;
  }

} _cityGraphNeighbor;

// Hash (rounded)
namespace std {
template <> struct hash<_cityGraphPoint> {
  std::size_t operator()(const _cityGraphPoint &point) const {
    double x = std::round(point.position.x / CELL_SIZE);
    double y = std::round(point.position.y / CELL_SIZE);
    double a = std::round(normalizeAngle(point.angle) / ANGLE_RESOLUTION);

    return std::hash<double>()(x) ^ std::hash<double>()(y) ^ std::hash<double>()(a);
  }
};
} // namespace std

namespace std {
template <> struct hash<_cityGraphNeighbor> {
  std::size_t operator()(const _cityGraphNeighbor &neighbor) const {
    return std::hash<_cityGraphPoint>()(neighbor.point) ^ std::hash<double>()(neighbor.maxSpeed) ^
           std::hash<double>()(neighbor.turningRadius) ^ std::hash<double>()(neighbor.distance);
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
  double getHeight() const { return height; }
  double getWidth() const { return width; }

private:
  std::unordered_map<point, std::vector<neighbor>> neighbors;
  std::unordered_set<point> graphPoints;

  void linkPoints(const point &point1, const point &point2);
  bool canLink(const point &point1, const point &point2, double speed, double *distance) const;

  double width;
  double height;
};
