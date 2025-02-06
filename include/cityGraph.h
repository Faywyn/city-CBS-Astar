#pragma once
#include <unordered_set>

#include "cityMap.h"
#include "config.h"
#include "utils.h"

typedef struct _cityGraphPoint {
  sf::Vector2f position;
  double angle;

  bool operator==(const _cityGraphPoint &other) const {
    int x = std::round(position.x / CELL_SIZE);
    int y = std::round(position.y / CELL_SIZE);
    int a = std::round(normalizeAngle(angle) / ANGLE_RESOLUTION);
    int oX = std::round(other.position.x / CELL_SIZE);
    int oY = std::round(other.position.y / CELL_SIZE);
    int oA = std::round(normalizeAngle(other.angle) / ANGLE_RESOLUTION);

    return x == oX && y == oY && a == oA;
  }
} _cityGraphPoint;

typedef struct _cityGraphNeighbor {
  _cityGraphPoint point;
  double maxSpeed;
  double turningRadius;
  double distance;
  bool isRightWay;

  bool operator==(const _cityGraphNeighbor &other) const {
    return point == other.point && maxSpeed == other.maxSpeed && turningRadius == other.turningRadius &&
           distance == other.distance && isRightWay == other.isRightWay;
  }

} _cityGraphNeighbor;

// Hash (rounded)
namespace std {
template <> struct hash<_cityGraphPoint> {
  std::size_t operator()(const _cityGraphPoint &point) const {
    int x = std::round(point.position.x / CELL_SIZE);
    int y = std::round(point.position.y / CELL_SIZE);
    int a = std::round(normalizeAngle(point.angle) / ANGLE_RESOLUTION);

    return std::hash<int>()(x) ^ std::hash<int>()(y) ^ std::hash<int>()(a);
  }
};
template <> struct hash<_cityGraphNeighbor> {
  std::size_t operator()(const _cityGraphNeighbor &neighbor) const {
    return std::hash<_cityGraphPoint>()(neighbor.point) ^ std::hash<double>()(neighbor.maxSpeed) ^
           std::hash<double>()(neighbor.turningRadius) ^ std::hash<double>()(neighbor.distance) ^
           std::hash<bool>()(neighbor.isRightWay);
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

  void linkPoints(const point &point1, const point &point2, int direction,
                  bool subPoints); // direction: 0 -> point1 to point2, 1 -> point2 to point1, 2 -> both
  bool canLink(const point &point1, const point &point2, double speed, double *distance) const;

  double width;
  double height;
};
