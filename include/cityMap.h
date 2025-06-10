/**
 * @file cityMap.h
 * @brief City map class definition
 *
 * This file contains the definition of the CityMap class, which represents a city map.
 */

#pragma once

#include "config.h"
#include <SFML/Graphics.hpp>
#include <math.h>
#include <string>
#include <tinyxml2.h>
#include <vector>

/**
 * @struct _cityMapSegment
 * @brief A segment in the city map
 */
typedef struct {
  sf::Vector2f p1;        /**< \brief The first point of the segment */
  sf::Vector2f p2;        /**< \brief The second point of the segment */
  sf::Vector2f p1_offset; /**< \brief The offset of the first point, used for the intersection */
  sf::Vector2f p2_offset; /**< \brief The offset of the second point, used for the intersection */
  sf::Angle angle;        /**< \brief The angle of the segment */
} _cityMapSegment;

/**
 * @struct _cityMapRoad
 * @brief A road in the city map
 */
typedef struct {
  int id;                                /**< \brief The id of the road */
  std::vector<_cityMapSegment> segments; /**< \brief The segments of the road */
  double width;                          /**< \brief The width of the road */
  int numLanes;                          /**< \brief The number of lanes of the road */
} _cityMapRoad;

/**
 * @struct _cityMapBuilding
 * @brief A building in the city map
 */
typedef struct {
  std::vector<sf::Vector2f> points; /**< \brief The points of the building */
} _cityMapBuilding;

/**
 * @struct _cityMapGreenArea
 * @brief A green area in the city map
 */
typedef struct {
  std::vector<sf::Vector2f> points; /**< \brief The points of the green area */
  int type;                         /**< \brief The type of the green area */
} _cityMapGreenArea;

/**
 * @struct _cityMapWaterArea
 * @brief A water area in the city map
 */
typedef struct {
  std::vector<sf::Vector2f> points; /**< \brief The points of the water area */
} _cityMapWaterArea;

/**
 * @struct _cityMapIntersection
 * @brief An intersection in the city map
 */
typedef struct {
  int id;                                          /**< \brief The id of the intersection */
  sf::Vector2f center;                             /**< \brief The center of the intersection */
  double radius;                                   /**< \brief The radius of the intersection */
  std::vector<std::pair<int, int>> roadSegmentIds; /**< \brief The ids of the road segments (roadId, segmentId). The
                                                       segments are the same for both directions of the road */
} _cityMapIntersection;

/**
 * @class CityMap
 * @brief A city map
 *
 * This class represents the city map. It contains the roads, intersections, buildings, green areas and water areas of
 * the city.
 */
class CityMap {
public:
  using segment = _cityMapSegment;
  using road = _cityMapRoad;
  using building = _cityMapBuilding;
  using greenArea = _cityMapGreenArea;
  using waterArea = _cityMapWaterArea;
  using intersection = _cityMapIntersection;

  /**
   * @brief Constructor
   */
  CityMap();

  /**
   * @brief Load a city map from a file
   * @param filename The filename
   */
  void loadFile(const std::string &filename);

  /**
   * @brief Check if the city map is loaded
   * @return True if the city map is loaded, false otherwise
   */
  bool isCityMapLoaded() const { return isLoaded; }

  /**
   * @brief Get the roads
   * @return The roads
   */
  std::vector<road> getRoads() const { return roads; }

  /**
   * @brief Get the intersections
   * @return The intersections
   */
  std::vector<intersection> getIntersections() const { return intersections; }

  /**
   * @brief Get the buildings
   * @return The buildings
   */
  std::vector<building> getBuildings() const { return buildings; }

  /**
   * @brief Get the green areas
   * @return The green areas
   */
  std::vector<greenArea> getGreenAreas() const { return greenAreas; }

  /**
   * @brief Get the water areas
   * @return The water areas
   */
  std::vector<waterArea> getWaterAreas() const { return waterAreas; }

  /**
   * @brief Get the minimum latitude and longitude
   * @return The minimum latitude and longitude
   */
  sf::Vector2f getMinLatLon() const { return minLatLon; }

  /**
   * @brief Get the maximum latitude and longitude
   * @return The maximum latitude and longitude
   */
  sf::Vector2f getMaxLatLon() const { return maxLatLon; }

  /**
   * @brief Get the width of the city map
   * @return The width of the city map
   */
  int getWidth() const { return width; }

  /**
   * @brief Get the height of the city map
   * @return The height of the city map
   */
  int getHeight() const { return height; }

private:
  bool isLoaded = false;

  std::vector<road> roads;
  std::vector<intersection> intersections;
  std::vector<building> buildings;
  std::vector<greenArea> greenAreas;
  std::vector<waterArea> waterAreas;

  sf::Vector2f minLatLon;
  sf::Vector2f maxLatLon;
  double width;  // in meters
  double height; // in meters
};
