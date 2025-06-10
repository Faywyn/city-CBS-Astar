/**
 * @file car.h
 * @brief A car in the city
 *
 * This file contains the declaration of the Car class. This class represents a car in the city. It contains the start
 * and end points of the car, the path of the car and the current point in the path.
 */
#pragma once

#include "aStar.h"
#include "cityGraph.h"
#include "dubins.h"
#include <vector>

/**
 * @class Car
 * @brief A car in the city
 *
 * This class represents a car in the city. It contains the start and end points of the car, the path of the car and
 * the current point in the path.
 */
class Car {
public:
  /**
   * @brief Constructor
   */
  Car();

  /**
   * @brief Assign the start and end points
   * @param start The start point
   * @param end The end point
   */
  void assignStartEnd(_cityGraphPoint start, _cityGraphPoint end) {
    this->start = start;
    this->end = end;
  }

  /**
   * @brief Choose a random start and end point in the graph
   * @param graph The graph
   * @param cityMap The city map
   */
  void chooseRandomStartEndPath(CityGraph &graph, CityMap &cityMap);

  /**
   * @brief Assign a path to the car
   * @param path The path
   */
  void assignPath(std::vector<AStar::node> path, CityGraph &graph);

  /**
   * @brief Assign an existing path to the car
   * @param path The path
   */
  void assignExistingPath(std::vector<sf::Vector2f> path);

  /**
   * @brief Move the car, move to the next point in the path
   */
  void move();

  /**
   * @brief Render the car
   * @param window The window
   */
  void render(sf::RenderWindow &window);

  /**
   * @brief Get the start point
   * @return The start point
   */
  _cityGraphPoint getStart() { return start; }

  /**
   * @brief Get the end point
   * @return The end point
   */
  _cityGraphPoint getEnd() { return end; }

  /**
   * @brief Get the current point in the path
   * @return The current point in the path
   */
  double getSpeed();

  /**
   * @brief Get the speed at a certain index in the path
   * @param index The index
   * @return The speed at the index
   */
  double getSpeedAt(int index);

  /**
   * @brief Get the average speed of the car
   * @param graph The graph
   * @return The average speed
   */
  double getAverageSpeed(CityGraph &graph);

  /**
   * @brief Get the remaining time to reach the end point
   * @return The remaining time
   */
  double getRemainingTime();

  /**
   * @brief Get the elapsed time since the start of the car
   * @return The elapsed time
   */
  double getElapsedTime();

  /**
   * @brief Get the time to reach the end point from the start point
   * @return The time
   */
  double getPathTime();

  /**
   * @brief Get the remaining distance to reach the end point
   * @return The remaining distance
   */
  double getRemainingDistance();

  /**
   * @brief Get the elapsed distance since the start of the car
   * @return The elapsed distance
   */
  double getElapsedDistance();

  /**
   * @brief Get the distance to reach the end point from the start point
   * @return The distance
   */
  double getPathLength();

  /**
   * @brief Get the position of the car
   * @return The position
   */
  sf::Vector2f getPosition() { return path[currentPoint]; }

  /**
   * @brief Get the path of the car
   * @return The path
   */
  std::vector<sf::Vector2f> getPath() { return path; }

  /**
   * @brief Get the path of the car from the A* algorithm
   * @return The path
   */
  std::vector<AStar::node> getAStarPath() { return aStarPath; }

  /**
   * @brief Toggle the debug mode. In debug mode, the path of the car is rendered
   * and the car is rendered in red
   */
  void toggleDebug() { debug = !debug; }

private:
  _cityGraphPoint start;
  _cityGraphPoint end;
  std::vector<sf::Vector2f> path;
  std::vector<AStar::node> aStarPath;
  int currentPoint = 0;
  bool debug = false;
  sf::Color color;
};

/**
 * @bref Check if two cars collided
 * @param car1 The first car
 * @param car2 The second car
 */
bool carsCollided(Car car1, Car car2, int time);

/**
 * @brief Check if two cars have a conflict
 * @param carPos The position of the car
 * @param carAngle The angle of the car
 * @param confPos The position of the conflicting car
 * @param confAngle The angle of the conflicting car
 * @return If the cars have a conflict
 */
bool carConflict(sf::Vector2f carPos, sf::Angle carAngle, sf::Vector2f confPos, sf::Angle confAngle);
