/**
 * @file aStar.h
 * @brief A* algorithm
 *
 * This file contains the A* algorithm. It is used to find the shortest path between two points in a graph.
 * It also contains the timed A* algorithm, which is used to find the shortest path between two points in a graph
 * while taking into account the constraints of the cars.
 */
#pragma once

#include "cityGraph.h"

/**
 * @struct _aStarNode
 * @brief A node for the A* algorithm
 *
 * This struct represents a node for the A* algorithm. It contains the point in the graph, the speed of the car
 * and the arc from which the node was reached.
 */
typedef struct _aStarNode {
  CityGraph::point point;                                   /**< \brief The point in the graph */
  double speed;                                             /**< \brief The speed of the car */
  std::pair<CityGraph::point, CityGraph::neighbor> arcFrom; /**< \brief The arc from which the node was reached */

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
  CityGraph::point point; /**< \brief The point in the graph */
  int time;               /**< \brief The time of the conflict */
  int car;                /**< \brief The car that caused the conflict */

  bool operator==(const _aStarConflict &other) const {
    return point == other.point && time == other.time && car == other.car;
  }
} _aStarConflict;

namespace std {
template <> struct hash<_aStarNode> {
  std::size_t operator()(const _aStarNode &point) const {
    double s = std::round(point.speed / SPEED_RESOLUTION);

    return std::hash<CityGraph::point>()(point.point) ^ std::hash<double>()(s) ^
           std::hash<CityGraph::point>()(point.arcFrom.first) ^ std::hash<CityGraph::neighbor>()(point.arcFrom.second);
  }
};
template <> struct hash<_aStarConflict> {
  std::size_t operator()(const _aStarConflict &conflict) const {
    return std::hash<CityGraph::point>()(conflict.point) ^ std::hash<int>()(conflict.time) ^
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

/**
 * @class ConstraintController
 * @brief Controller for constraints
 *
 * This class is used to control the constraints of the A* algorithm. It is used to check if a car can move to a certain
 * point in the graph at a certain time.
 */
class ConstraintController {
public:
  /**
   * @brief Constructor
   */
  ConstraintController() { this->constraints.clear(); }

  /**
   * @brief Copy constructor
   * @return A copy of the object
   */
  ConstraintController copy();

  /**
   * @brief Copy constructor
   * @param cars The cars to copy
   * @return A copy of the object
   */
  ConstraintController copy(std::vector<int> cars);

  /**
   * @brief Add a constraint
   * @param constraints The constraint to add
   */
  void addConstraint(AStar::conflict constraints);

  /**
   * @brief Check if a constraint exists
   * @param constraint The constraint to check
   * @return True if the constraint exists, false otherwise
   */
  bool hasConstraint(AStar::conflict constraint);

  /**
   * @brief Check if a car can move to a certain point in the graph at a certain time
   * @param car The car
   * @param speed The speed of the car
   * @param newSpeed The new speed of the car
   * @param time The time
   * @param from The point from which the car is moving
   * @param to The point to which the car is moving
   * @return True if the car can move to the point, false otherwise
   */
  bool checkConstraints(int car, double speed, double newSpeed, double time, CityGraph::point from,
                        CityGraph::neighbor to);

private:
  std::vector<std::vector<std::vector<AStar::conflict>>> constraints; // [car][time][constraints]
};

/**
 * @class TimedAStar
 * @brief Timed A* algorithm
 *
 * This class represents the timed A* algorithm. It is used to find the shortest path between two points in a graph
 * while taking into account the constraints of the cars.
 */
class TimedAStar {
public:
  /**
   * @brief Constructor
   * @param start The start point
   * @param end The end point
   * @param cityGraph The graph
   * @param constraints The constraints
   * @param carIndex The car index
   */
  TimedAStar(CityGraph::point start, CityGraph::point end, const CityGraph &cityGraph,
             ConstraintController *constraints, int carIndex);

  /**
   * @brief Find the path
   * @return The path
   */
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
  ConstraintController *conflicts;
  int carIndex;
  CityGraph graph;

  void process();
};
