/**
 * @file cityMap.cpp
 * @brief CityMap class implementation
 *
 * This file contains the implementation of the CityMap class.
 */
#include <iostream>
#include <math.h>
#include <set>

#include "spdlog/spdlog.h"
#include "tinyxml2.h"

#include "cityMap.h"
#include "utils.h"

CityMap::CityMap() {
  roads.clear();
  intersections.clear();
  minLatLon.x = minLatLon.y = maxLatLon.x = maxLatLon.y = 0;
}

void CityMap::loadFile(const std::string &filename) {
  spdlog::info("Loading file: {}", filename);

  tinyxml2::XMLDocument doc;
  // Load the XML file
  if (doc.LoadFile(filename.c_str()) != tinyxml2::XML_SUCCESS) {
    spdlog::error("Failed to load file: {}", filename);
    return;
  }

  // Extract the bounds of the map
  tinyxml2::XMLElement *bounds = doc.FirstChildElement("osm")->FirstChildElement("bounds");
  if (!bounds) {
    spdlog::error("Failed to extract bounds from file: {}", filename);
    return;
  }

  minLatLon.x = bounds->FloatAttribute("minlon");
  minLatLon.y = bounds->FloatAttribute("minlat");
  maxLatLon.x = bounds->FloatAttribute("maxlon");
  maxLatLon.y = bounds->FloatAttribute("maxlat");

  // Define the width and height of the map
  width = latLonToXY(minLatLon.y, minLatLon.x).x - latLonToXY(maxLatLon.y, maxLatLon.x).x;
  height = latLonToXY(minLatLon.y, minLatLon.x).y - latLonToXY(maxLatLon.y, maxLatLon.x).y;
  width = std::abs(width);
  height = std::abs(height);

  std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
  spdlog::info("Loading roads and buildings ...");

  // List of highway types to exclude
  std::set<std::string> excludedHighways = {"footway", "path",  "pedestrian", "cycleway",
                                            "steps",   "track", "bridleway",  "service"};

  // List of highway types to include
  std::set<std::string> includedHighways = {
      "motorway",      "trunk",         "primary",    "secondary",    "tertiary",       "unclassified", "residential",
      "living_street", "motorway_link", "trunk_link", "primary_link", "secondary_link", "tertiary_link"};

  // Extract the roads
  tinyxml2::XMLElement *way = doc.FirstChildElement("osm")->FirstChildElement("way");
  int roadId = 0;
  while (way) {
    road r;
    building b;
    greenArea g;
    waterArea w;
    r.width = DEFAULT_ROAD_WIDTH;
    r.numLanes = r.width / DEFAULT_LANE_WIDTH;
    r.id = roadId;

    tinyxml2::XMLElement *nd = way->FirstChildElement("nd");
    while (nd) {
      tinyxml2::XMLElement *node = doc.FirstChildElement("osm")->FirstChildElement("node");
      while (node) {
        if (node->IntAttribute("id") == nd->IntAttribute("ref")) {
          sf::Vector2f p;
          p.x = node->FloatAttribute("lon");
          p.y = node->FloatAttribute("lat");

          if (r.segments.size() > 0) {
            segment s;
            s.p1 = r.segments.back().p2;
            s.p2 = p;
            r.segments.push_back(s);
          } else {
            segment s;
            s.p1 = p;
            s.p2 = p;
            r.segments.push_back(s);
          }

          b.points.push_back(p);
          g.points.push_back(p);
          w.points.push_back(p);
          break;
        }
        node = node->NextSiblingElement("node");
      }
      nd = nd->NextSiblingElement("nd");
    }

    // Remove the first segment (it has the same p1 and p2)
    r.segments.erase(r.segments.begin());

    std::string highwayType;
    bool isHighway = false;
    bool isBuilding = false;
    bool isUnderground = false;
    bool isGreenArea = false;
    bool isWaterArea = false;
    bool widthSet = false;
    bool lanesSet = false;
    tinyxml2::XMLElement *tag = way->FirstChildElement("tag");
    while (tag) {
      if (strcmp(tag->Attribute("k"), "width") == 0) {
        r.width = tag->FloatAttribute("v");
        widthSet = true;
      } else if (strcmp(tag->Attribute("k"), "lanes") == 0) {
        r.numLanes = tag->IntAttribute("v");
        lanesSet = true;
      } else if (strcmp(tag->Attribute("k"), "highway") == 0) {
        highwayType = tag->Attribute("v");
        isHighway = true;
      } else if (strcmp(tag->Attribute("k"), "building") == 0) {
        isBuilding = true;
      } else if (strcmp(tag->Attribute("k"), "layer") == 0) {
        int layerValue = tag->IntAttribute("v");
        if (layerValue < 0) {
          isUnderground = true;
        }
      } else if (strcmp(tag->Attribute("k"), "landuse") == 0) {
        if (strcmp(tag->Attribute("v"), "forest") == 0 || strcmp(tag->Attribute("v"), "grass") == 0 ||
            strcmp(tag->Attribute("v"), "meadow") == 0) {
          isGreenArea = true;
          g.type = 0;
        }
      } else if (strcmp(tag->Attribute("k"), "leisure") == 0) {
        if (strcmp(tag->Attribute("v"), "park") == 0 || strcmp(tag->Attribute("v"), "garden") == 0) {
          isGreenArea = true;
          g.type = 1;
        }
      } else if (strcmp(tag->Attribute("k"), "waterway") == 0 &&
                 (strcmp(tag->Attribute("v"), "river") == 0 || strcmp(tag->Attribute("v"), "stream") == 0 ||
                  strcmp(tag->Attribute("v"), "canal") == 0)) {
        isWaterArea = true;
      } else if (strcmp(tag->Attribute("k"), "natural") == 0 &&
                 (strcmp(tag->Attribute("v"), "water") == 0 || strcmp(tag->Attribute("v"), "wetland") == 0)) {
        isWaterArea = true;
      } else if (strcmp(tag->Attribute("k"), "water") == 0 &&
                 (strcmp(tag->Attribute("v"), "lake") == 0 || strcmp(tag->Attribute("v"), "pond") == 0 ||
                  strcmp(tag->Attribute("v"), "river") == 0)) {
        isWaterArea = true;
      }
      tag = tag->NextSiblingElement("tag");
    }
    if (!widthSet && !lanesSet) {
      r.width = DEFAULT_ROAD_WIDTH;
      r.numLanes = r.width / DEFAULT_LANE_WIDTH;
    } else if (!widthSet) {
      r.width = r.numLanes * DEFAULT_LANE_WIDTH;
    } else if (!lanesSet) {
      r.numLanes = r.width / DEFAULT_LANE_WIDTH;
    }
    r.width = std::max(r.width, MIN_ROAD_WIDTH);
    r.numLanes = std::max(r.numLanes, 1);

    if (isUnderground) {
      way = way->NextSiblingElement("way");
      continue;
    }
    if (isBuilding) {
      buildings.push_back(b);
      way = way->NextSiblingElement("way");
      continue;
    }
    if (isGreenArea) {
      greenAreas.push_back(g);
      way = way->NextSiblingElement("way");
      continue;
    }
    if (isWaterArea) {
      waterAreas.push_back(w);
      way = way->NextSiblingElement("way");
      continue;
    }
    if (!isHighway || excludedHighways.find(highwayType) != excludedHighways.end()) {
      way = way->NextSiblingElement("way");
      continue;
    }
    if (includedHighways.find(highwayType) != includedHighways.end()) {
      roads.push_back(r);
      roadId++;
    }

    way = way->NextSiblingElement("way");
  }

  // Convert lat/lon to meters (using the upper-left corner as origin)
  sf::Vector2f minXY = latLonToXY(minLatLon.y, minLatLon.x);
  sf::Vector2f maxXY = latLonToXY(maxLatLon.y, maxLatLon.x);
  for (auto &r : roads) {
    for (auto &s : r.segments) {
      s.p1 = latLonToXY(s.p1.y, s.p1.x);
      s.p2 = latLonToXY(s.p2.y, s.p2.x);

      s.p1.x -= minXY.x;
      s.p1.y -= minXY.y;
      s.p2.x -= minXY.x;
      s.p2.y -= minXY.y;

      // Symetri to the x-axis
      s.p1.y = maxXY.y - minXY.y - s.p1.y;
      s.p2.y = maxXY.y - minXY.y - s.p2.y;

      s.p1_offset = s.p1;
      s.p2_offset = s.p2;

      s.angle = sf::radians(std::atan2(s.p2.y - s.p1.y, s.p2.x - s.p1.x));
    }
  }
  for (auto &b : buildings) {
    for (auto &p : b.points) {
      p = latLonToXY(p.y, p.x);

      p.x -= minXY.x;
      p.y -= minXY.y;

      // Symetri to the x-axis
      p.y = maxXY.y - minXY.y - p.y;
    }
  }
  for (auto &g : greenAreas) {
    for (auto &p : g.points) {
      p = latLonToXY(p.y, p.x);

      p.x -= minXY.x;
      p.y -= minXY.y;

      // Symetri to the x-axis
      p.y = maxXY.y - minXY.y - p.y;
    }
  }
  for (auto &w : waterAreas) {
    for (auto &p : w.points) {
      p = latLonToXY(p.y, p.x);

      p.x -= minXY.x;
      p.y -= minXY.y;

      // Symetri to the x-axis
      p.y = maxXY.y - minXY.y - p.y;
    }
  }

  std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
  spdlog::info("Roads and buildings loaded ({} ms)",
               std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count());

  spdlog::info("Loading intersections ...");

  // Intersections are at any roads' points if they are near another one
  // First add the intersections for each node point
  // Then merge the intersections that are close to each other
  intersections.clear();
  int intersectionId = 0;

  // Add the intersections for each road segment
  spdlog::debug("Adding intersections ...");
  for (auto r : roads) {
    for (int s_id = 0; s_id < (int)r.segments.size(); s_id++) {
      segment s = r.segments[s_id];
      std::vector<sf::Vector2f> points = {s.p1, s.p2};
      for (auto p : points) {
        intersection i = {intersectionId++, p, r.width / 2, {}};
        i.roadSegmentIds.push_back({r.id, s_id});
        intersections.push_back(i);
      }
    }
  }
  spdlog::debug("Intersections added");

  // Merge the intersections that are close to each other
  spdlog::debug("Merging intersections ...");
  for (int distCoef = 5; distCoef > 0; distCoef -= 1) {
    for (int i = 0; i < (int)intersections.size(); i++) {
      for (int j = i + 1; j < (int)intersections.size(); j++) {
        bool is_i = intersections[i].roadSegmentIds.size() > intersections[j].roadSegmentIds.size();

        if (intersections[i].roadSegmentIds.size() == intersections[j].roadSegmentIds.size()) {
          is_i = intersections[i].id < intersections[j].id;
        }

        double minSpace = intersections[i].radius + intersections[j].radius;
        minSpace /= distCoef;

        if (distance(intersections[i].center, intersections[j].center) < minSpace) {
          // Merge the intersections to i or j (depending on is_i)
          int index_from = is_i ? j : i;
          int index_to = is_i ? i : j;

          for (auto &r : intersections[index_from].roadSegmentIds) {
            intersections[index_to].roadSegmentIds.push_back(r);
          }

          intersections.erase(intersections.begin() + index_from);
          i -= 1;
          break;
        }
      }
    }
  }
  spdlog::debug("Intersections merged");

  // Make the road point to be outside the intersection
  spdlog::debug("Adding offsets to the roads ...");
  for (auto &i : intersections) {
    for (auto &roadInfo : i.roadSegmentIds) {
      double dx =
          roads[roadInfo.first].segments[roadInfo.second].p2.x - roads[roadInfo.first].segments[roadInfo.second].p1.x;
      double dy =
          roads[roadInfo.first].segments[roadInfo.second].p2.y - roads[roadInfo.first].segments[roadInfo.second].p1.y;
      double dd = distance({0, 0}, {(float)dx, (float)dy});
      dx /= dd;
      dy /= dd;

      double radius = i.radius;

      if (distance(roads[roadInfo.first].segments[roadInfo.second].p1, i.center) <
          distance(roads[roadInfo.first].segments[roadInfo.second].p2, i.center)) {
        roads[roadInfo.first].segments[roadInfo.second].p1_offset.x = i.center.x + dx * radius;
        roads[roadInfo.first].segments[roadInfo.second].p1_offset.y = i.center.y + dy * radius;
      } else {
        dx = -dx;
        dy = -dy;
        roads[roadInfo.first].segments[roadInfo.second].p2_offset.x = i.center.x + dx * radius;
        roads[roadInfo.first].segments[roadInfo.second].p2_offset.y = i.center.y + dy * radius;
      }
    }
  }
  spdlog::debug("Offsets added");

  // Remove the intersections that link the same road
  spdlog::debug("Removing intersections that link the same road ...");
  for (int i = 0; i < (int)intersections.size(); i++) {
    if (intersections[i].roadSegmentIds.size() != 2)
      continue;

    if (intersections[i].roadSegmentIds[0].first == intersections[i].roadSegmentIds[1].first) {
      intersections.erase(intersections.begin() + i);
      i -= 1;
    }
  }
  spdlog::debug("Intersections removed");

  // Log all the intersections and roads
  for (auto r : roads) {
    spdlog::debug("Road: id={}, width={}, numLanes={}, segments={}", r.id, r.width, r.numLanes, r.segments.size());
  }
  for (auto i : intersections) {
    spdlog::debug("Intersection: id={}, center=({}, {}), radius={}, roadSegmentIds={}", i.id, i.center.x, i.center.y,
                  i.radius, i.roadSegmentIds.size());
  }

  std::chrono::steady_clock::time_point end2 = std::chrono::steady_clock::now();
  spdlog::info("Intersections loaded ({} ms)",
               std::chrono::duration_cast<std::chrono::milliseconds>(end2 - end).count());

  spdlog::info("Number of roads: {}", roads.size());
  spdlog::info("Number of buildings: {}", buildings.size());
  spdlog::info("Number of intersections: {}", intersections.size());

  spdlog::info("Width: {} m", width);
  spdlog::info("Height: {} m", height);

  isLoaded = true;
}
