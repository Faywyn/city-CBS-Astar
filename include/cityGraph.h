/**
 * @file cityGraph.h
 * @brief A graph representing the city's streets and intersections using a graph.
 *
 * This file contains the definition of the CityGraph class.
 */
#pragma once
#include <unordered_set>

#include "cityMap.h"
#include "config.h"
#include "utils.h"

/**
 * @struct _cityGraphPoint
 * @brief A point in the city graph
 *
 * This struct represents a point in the city graph. It contains the position and the angle of the point.
 */
typedef struct _cityGraphPoint {
  sf::Vector2f position; /**< \brief The position of the point */
  double angle;          /**< \brief The angle of the point */

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

/**
 * @struct _cityGraphNeighbor
 * @brief A neighbor of a point in the city graph
 *
 * This struct represents a neighbor of a point in the city graph. It contains the neighbor point, the maximum speed to
 * reach it, the turning radius to reach it, the distance to reach it and if it is the right way.
 */
typedef struct _cityGraphNeighbor {
  _cityGraphPoint point; /**< \brief The neighbor point */
  double maxSpeed;       /**< \brief The maximum speed to reach the neighbor point */
  double turningRadius;  /**< \brief The turning radius to reach the neighbor point */
  double distance;       /**< \brief The distance to reach the neighbor point */
  bool isRightWay;       /**< \brief If it is the right way */

  bool operator==(const _cityGraphNeighbor &other) const {
    return point == other.point && maxSpeed == other.maxSpeed && turningRadius == other.turningRadius &&
           distance == other.distance && isRightWay == other.isRightWay;
  }

} _cityGraphNeighbor;

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

/**
 * @class CityGraph
 * @brief A graph representing the city's streets and intersections using a graph.
 *
 * This class represents the city graph. It contains the neighbors of each point in the graph and the graph points.
 */
class CityGraph {
public:
  using point = _cityGraphPoint;
  using neighbor = _cityGraphNeighbor;

  /**
   * @brief Create a city graph
   *
   * This constructor creates a city graph from a city map.
   *
   * @param cityMap The city map
   */
  void createGraph(const CityMap &cityMap);

  /**
   * @brief Get neighbors map
   * @return Neighbors map
   */
  std::unordered_map<point, std::vector<neighbor>> getNeighbors() const { return neighbors; }

  /**
   * @brief Get graph points
   * @return Graph points
   */
  std::unordered_set<point> getGraphPoints() const { return graphPoints; }

  /**
   * @brief Get random point
   * @return Random point
   */
  point getRandomPoint() const;

  /**
   * @brief Get the height of the city graph
   * @return The height of the city graph
   */
  double getHeight() const { return height; }

  /**
   * @brief Get the width of the city graph
   * @return The width of the city graph
   */
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
