/**
 * @file aStar.h
 * @brief A* algorithm
 *
 * This file contains the declaration of the AStar class. This class represents the A* algorithm, which is used to find
 * the shortest path between two points in a graph.
 */
#pragma once

#include "cityGraph.h"
#include "config.h"

/**
 * @struct _aStarNode
 * @brief A node for the A* algorithm
 *
 * This struct represents a node for the A* algorithm. It contains the point in the graph, the speed of the car
 * and the arc from which the node was reached.
 */
typedef struct _aStarNode {
  _cityGraphPoint point;                                  /**< \brief The point in the graph */
  double speed;                                           /**< \brief The speed of the car */
  std::pair<_cityGraphPoint, _cityGraphNeighbor> arcFrom; /**< \brief The arc from which the node was reached */

  bool operator==(const _aStarNode &other) const {
    double s = std::round(speed / SPEED_RESOLUTION);
    double oS = std::round(other.speed / SPEED_RESOLUTION);

    return point == other.point && s == oS && arcFrom.first == other.arcFrom.first &&
           arcFrom.second == other.arcFrom.second;
  }
} _aStarNode;

/**
 * @struct _aStarConflict
 * @brief A conflict for the A* algorithm
 *
 * This struct represents a conflict for the A* algorithm. It contains the point in the graph, the time of the conflict
 * and the car that caused the conflict.
 */
typedef struct _aStarConflict {
  _cityGraphPoint point; /**< \brief The point in the graph */
  int time;              /**< \brief The time of the conflict */
  int car;               /**< \brief The car that caused the conflict */

  bool operator==(const _aStarConflict &other) const {
    return point == other.point && time == other.time && car == other.car;
  }
} _aStarConflict;

namespace std {
template <> struct hash<_aStarNode> {
  std::size_t operator()(const _aStarNode &point) const {
    double s = std::round(point.speed / SPEED_RESOLUTION);

    return std::hash<_cityGraphPoint>()(point.point) ^ std::hash<double>()(s) ^
           std::hash<_cityGraphPoint>()(point.arcFrom.first) ^ std::hash<CityGraph::neighbor>()(point.arcFrom.second);
  }
};
template <> struct hash<_aStarConflict> {
  std::size_t operator()(const _aStarConflict &conflict) const {
    return std::hash<_cityGraphPoint>()(conflict.point) ^ std::hash<int>()(conflict.time) ^
           std::hash<int>()(conflict.car);
  }
};
} // namespace std

/**
 * @class AStar
 * @brief A* algorithm
 *
 * This class represents the A* algorithm. It is used to find the shortest path between two points in a graph.
 */
class AStar {
public:
  using node = _aStarNode;
  using conflict = _aStarConflict;

  /**
   * @brief Constructor
   * @param start The start point
   * @param end The end point
   * @param cityGraph The graph
   */
  AStar(CityGraph::point start, CityGraph::point end, const CityGraph &cityGraph);

  /**
   * @brief Find the path
   * @return The path
   */
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
