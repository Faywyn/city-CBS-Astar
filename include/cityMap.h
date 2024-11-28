#pragma once

#include <SFML/Graphics.hpp>
#include <string>
#include <vector>

typedef struct {
  sf::Vector2f p1;
  sf::Vector2f p2;
  sf::Vector2f p1_offset; // For intersections radius
  sf::Vector2f p2_offset; // For intersections radius
  float angle;            // in rad, between lines (p2.x - p1.x, p2.y - p1.y) and (1, 0)
} segment;

typedef struct {
  int id;
  std::vector<segment> segments;
  float width;
  int numLanes;
} road;

typedef struct {
  std::vector<sf::Vector2f> points;
} building;

typedef struct {
  int id;
  sf::Vector2f center;
  float radius;
  std::vector<std::pair<int, int>> roadSegmentIds; // (roadId, segmentId)
                                                   // so their will be 2 segments per road (for each direction)
} intersection;

class CityMap {
public:
  CityMap();
  ~CityMap();

  void loadFile(const std::string &filename);
  bool isCityMapLoaded() const { return isLoaded; }

  // Getters
  std::vector<road> getRoads() const { return roads; }
  std::vector<intersection> getIntersections() const { return intersections; }
  std::vector<building> getBuildings() const { return buildings; }
  sf::Vector2f getMinLatLon() const { return minLatLon; }
  sf::Vector2f getMaxLatLon() const { return maxLatLon; }

  int getWidth() const { return width; }
  int getHeight() const { return height; }

private:
  bool isLoaded = false;

  std::vector<road> roads;
  std::vector<intersection> intersections;
  std::vector<building> buildings;

  sf::Vector2f minLatLon;
  sf::Vector2f maxLatLon;
  float width;  // in meters
  float height; // in meters
};
