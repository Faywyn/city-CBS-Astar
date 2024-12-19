#include <iostream>
#include <ompl/base/State.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <random>
#include <spdlog/spdlog.h>

#include "cityGraph.h"
#include "config.h"
#include "utils.h"

namespace ob = ompl::base;

void CityGraph::createGraph(const CityMap &cityMap) {
  auto roads = cityMap.getRoads();
  auto intersections = cityMap.getIntersections();

  this->height = cityMap.getHeight();
  this->width = cityMap.getWidth();

  // Graph's points are evenly distributed along a road segment
  for (const auto &road : roads) {
    if (road.segments.empty()) {
      continue;
    }

    int numSeg = 0;
    for (const auto &segment : road.segments) {
      if (numSeg > 0) { // Link to the previous one
        for (int i_lane = 0; i_lane < road.numLanes; i_lane++) {
          double offset = ((double)i_lane - (double)road.numLanes / 2.0f) * road.width / road.numLanes;
          offset += road.width / (2 * road.numLanes);

          point point1;
          point1.angle = road.segments[numSeg - 1].angle;
          point1.position =
              sf::Vector2f(road.segments[numSeg - 1].p2_offset.x + offset * sin(road.segments[numSeg - 1].angle),
                           road.segments[numSeg - 1].p2_offset.y + offset * -cos(road.segments[numSeg - 1].angle));

          point point2;
          point2.angle = road.segments[numSeg].angle;
          point2.position =
              sf::Vector2f(road.segments[numSeg].p1_offset.x + offset * sin(road.segments[numSeg].angle),
                           road.segments[numSeg].p1_offset.y + offset * -cos(road.segments[numSeg].angle));

          linkPoints(point1, point2);
        }
      }
      numSeg++;

      double segmentLength =
          sqrt(pow(segment.p2_offset.x - segment.p1_offset.x, 2) + pow(segment.p2_offset.y - segment.p1_offset.y, 2));
      double pointDistance = 15;
      int numPoints = segmentLength / pointDistance;
      double dx_s = (segment.p2_offset.x - segment.p1_offset.x) / numPoints;
      double dy_s = (segment.p2_offset.y - segment.p1_offset.y) / numPoints;
      double dx_a = sin(segment.angle);
      double dy_a = -cos(segment.angle);

      for (int i_lane = 0; i_lane < road.numLanes; i_lane++) {
        double offset = ((double)i_lane - (double)road.numLanes / 2.0f) * road.width / road.numLanes;
        offset += road.width / (2 * road.numLanes);

        if (numPoints == 0) {
          point point1;
          point1.angle = segment.angle;
          point1.position = sf::Vector2f(segment.p1_offset.x + offset * dx_a, segment.p1_offset.y + offset * dy_a);

          point point2;
          point2.angle = segment.angle;
          point2.position = sf::Vector2f(segment.p2_offset.x + offset * dx_a, segment.p2_offset.y + offset * dy_a);

          linkPoints(point1, point2);
          continue;
        }

        for (int i = 0; i <= numPoints; i++) {
          point point1;
          point1.position = sf::Vector2f(segment.p1_offset.x + i * dx_s + offset * dx_a,
                                         segment.p1_offset.y + i * dy_s + offset * dy_a);
          point1.angle = segment.angle;

          if (i > 0) {
            for (int i2_lane = 0; i2_lane < road.numLanes; i2_lane++) {
              double offset2 = ((double)i2_lane - (double)road.numLanes / 2.0f) * road.width / road.numLanes;
              offset2 += road.width / (2 * road.numLanes);

              point point2;
              point2.position = sf::Vector2f(segment.p1_offset.x + (i - 1) * dx_s + offset2 * dx_a,
                                             segment.p1_offset.y + (i - 1) * dy_s + offset2 * dy_a);
              point2.angle = segment.angle;

              linkPoints(point1, point2);
            }
          }
        }
      }
    }
  }

  // Connect the intersections
  for (const auto &intersection : intersections) {
    for (const auto &roadSegmentId1 : intersection.roadSegmentIds) {
      for (const auto &roadSegmentId2 : intersection.roadSegmentIds) {
        const auto &road1 = roads[roadSegmentId1.first];
        const auto &road2 = roads[roadSegmentId2.first];
        const auto &segment1 = road1.segments[roadSegmentId1.second];
        const auto &segment2 = road2.segments[roadSegmentId2.second];

        // Find the point of the segment2 closest to the intersection
        point point1;
        point1.angle = segment1.angle;
        point1.position = (distance(segment1.p1, intersection.center) < distance(segment1.p2, intersection.center))
                              ? segment1.p1_offset
                              : segment1.p2_offset;

        point point2;
        point2.angle = segment2.angle;
        point2.position = (distance(segment2.p1, intersection.center) < distance(segment2.p2, intersection.center))
                              ? segment2.p1_offset
                              : segment2.p2_offset;

        for (int iL_1 = 0; iL_1 < road1.numLanes; iL_1++) {
          double offset1 = ((double)iL_1 - (double)road1.numLanes / 2.0f) * road1.width / road1.numLanes;
          offset1 += road1.width / (2 * road1.numLanes);

          for (int iL_2 = 0; iL_2 < road2.numLanes; iL_2++) {
            double offset2 = ((double)iL_2 - (double)road2.numLanes / 2.0f) * road2.width / road2.numLanes;
            offset2 += road2.width / (2 * road2.numLanes);

            point point1_offset;
            point1_offset.angle = segment1.angle;
            point1_offset.position = sf::Vector2f(point1.position.x + offset1 * sin(segment1.angle),
                                                  point1.position.y + offset1 * -cos(segment1.angle));

            point point2_offset;
            point2_offset.angle = segment2.angle;
            point2_offset.position = sf::Vector2f(point2.position.x + offset2 * sin(segment2.angle),
                                                  point2.position.y + offset2 * -cos(segment2.angle));

            linkPoints(point1_offset, point2_offset);
          }
        }
      }
    }
  }

  spdlog::info("Graph created with {} points", graphPoints.size());

  // Remove all the neighbors that need to turn too much
  for (auto &point : graphPoints) {
    std::vector<neighbor> newNeighbors;
    double distance;
    for (auto &neighbor : neighbors[point]) {
      double speed = turningRadiusToSpeed(CAR_MIN_TURNING_RADIUS);
      bool can = canLink(point, neighbor.point, speed, &distance);

      if (!can)
        continue;

      while (canLink(point, neighbor.point, speed + 0.1, &distance)) {
        speed += 0.1;
        if (speed >= CAR_MAX_SPEED_MS) {
          speed = CAR_MAX_SPEED_MS;
          break;
        }
      }

      if (can) {
        neighbor.maxSpeed = speed;
        // spdlog::warn("In cityGraph.cpp: restore the valid distance calculation");
        neighbor.distance = std::sqrt(std::pow(neighbor.point.position.x - point.position.x, 2) +
                                      std::pow(neighbor.point.position.y - point.position.y, 2));

        // neighbor.distance = std::min(neighbor.distance * 1.3, distance);

        std::cout << distance << " - " << neighbor.distance << std::endl;

        neighbor.turningRadius = turningRadius(speed);
        newNeighbors.push_back(neighbor);
      }
    }

    neighbors[point].clear();
    for (const auto &neighbor : newNeighbors) {
      neighbors[point].push_back(neighbor);
    }
  }
}

void CityGraph::linkPoints(const point &p, const point &n) {
  std::vector<double> anglesPoint = {normalizeAngle(p.angle), normalizeAngle(p.angle + M_PI)};
  std::vector<double> anglesNeighbor = {normalizeAngle(n.angle), normalizeAngle(n.angle + M_PI)};

  point copyPoint = p;
  point copyNeighbor = n;

  for (const auto &anglePoint : anglesPoint) {
    for (const auto &angleNeighbor : anglesNeighbor) {
      copyPoint.angle = anglePoint;
      copyNeighbor.angle = angleNeighbor;

      neighbors[copyPoint].push_back({copyNeighbor, 0, 0, 0}); // This fields will be updated later
      neighbors[copyNeighbor].push_back({copyPoint, 0, 0, 0});

      graphPoints.insert(copyPoint);
      graphPoints.insert(copyNeighbor);
    }
  }
}

CityGraph::point CityGraph::getRandomPoint() const {
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<> dis(0, graphPoints.size() - 1);

  auto it = graphPoints.begin();
  std::advance(it, dis(gen));

  return *it;
}

bool CityGraph::canLink(const point &point1, const point &point2, double speed, double *distance) const {
  double radius = turningRadius(speed);

  ob::DubinsStateSpace space(radius);

  ob::State *start = space.allocState();
  ob::State *end = space.allocState();

  start->as<ob::DubinsStateSpace::StateType>()->setXY(point1.position.x, point1.position.y);
  start->as<ob::DubinsStateSpace::StateType>()->setYaw(point1.angle);

  end->as<ob::DubinsStateSpace::StateType>()->setXY(point2.position.x, point2.position.y);
  end->as<ob::DubinsStateSpace::StateType>()->setYaw(point2.angle);

  double total = 0;

  // Extract the path
  ob::DubinsStateSpace::DubinsPath path = space.dubins(start, end);
  for (unsigned int i = 0; i < 3; ++i) // Dubins path has up to 3 segments
  {
    auto type = path.type_[i];
    if (type == ob::DubinsStateSpace::DubinsPathSegmentType::DUBINS_LEFT) {
      total += std::abs(path.length_[i]);
    } else if (type == ob::DubinsStateSpace::DubinsPathSegmentType::DUBINS_RIGHT) {
      total += std::abs(path.length_[i]);
    }
  }

  *distance = space.distance(start, end);
  return total < M_PI * 0.75f;
}
