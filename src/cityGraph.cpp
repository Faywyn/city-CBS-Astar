#include <iostream>
#include <ompl/base/State.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <spdlog/spdlog.h>

#include "cityGraph.h"
#include "utils.h"

namespace ob = ompl::base;

void CityGraph::createGraph(const CityMap &cityMap) {
  auto roads = cityMap.getRoads();
  auto intersections = cityMap.getIntersections();

  // Graph's points are evenly distributed along a road segment
  for (const auto &road : roads) {
    if (road.segments.empty()) {
      continue;
    }

    int numSeg = 0;
    for (const auto &segment : road.segments) {
      if (numSeg > 0) { // Link to the previous one
        for (int i_lane = 0; i_lane < road.numLanes; i_lane++) {
          float offset = ((float)i_lane - (float)road.numLanes / 2.0f) * road.width / road.numLanes;
          offset += road.width / (2 * road.numLanes);

          graphPoint point1;
          point1.angle = road.segments[numSeg - 1].angle;
          point1.position =
              sf::Vector2f(road.segments[numSeg - 1].p2_offset.x + offset * sin(road.segments[numSeg - 1].angle),
                           road.segments[numSeg - 1].p2_offset.y + offset * -cos(road.segments[numSeg - 1].angle));

          graphPoint point2;
          point2.angle = road.segments[numSeg].angle;
          point2.position =
              sf::Vector2f(road.segments[numSeg].p1_offset.x + offset * sin(road.segments[numSeg].angle),
                           road.segments[numSeg].p1_offset.y + offset * -cos(road.segments[numSeg].angle));

          linkPoints(point1, point2);
        }
      }
      numSeg++;

      float segmentLength =
          sqrt(pow(segment.p2_offset.x - segment.p1_offset.x, 2) + pow(segment.p2_offset.y - segment.p1_offset.y, 2));
      float pointDistance = TURNING_RADIUS < 7 ? 7 : (TURNING_RADIUS > 15 ? 15 : TURNING_RADIUS);
      int numPoints = segmentLength / pointDistance;
      float dx_s = (segment.p2_offset.x - segment.p1_offset.x) / numPoints;
      float dy_s = (segment.p2_offset.y - segment.p1_offset.y) / numPoints;
      float dx_a = sin(segment.angle);
      float dy_a = -cos(segment.angle);

      for (int i_lane = 0; i_lane < road.numLanes; i_lane++) {
        float offset = ((float)i_lane - (float)road.numLanes / 2.0f) * road.width / road.numLanes;
        offset += road.width / (2 * road.numLanes);

        if (numPoints == 0) {
          graphPoint point1;
          point1.angle = segment.angle;
          point1.position = sf::Vector2f(segment.p1_offset.x + offset * dx_a, segment.p1_offset.y + offset * dy_a);

          graphPoint point2;
          point2.angle = segment.angle;
          point2.position = sf::Vector2f(segment.p2_offset.x + offset * dx_a, segment.p2_offset.y + offset * dy_a);

          linkPoints(point1, point2);
          continue;
        }

        for (int i = 0; i <= numPoints; i++) {
          graphPoint point;
          point.position = sf::Vector2f(segment.p1_offset.x + i * dx_s + offset * dx_a,
                                        segment.p1_offset.y + i * dy_s + offset * dy_a);
          point.angle = segment.angle;

          if (i > 0) {
            if (i == 1 || i == numPoints || i % 3 == 0) { // Connect to the previous on the other lane
              for (int i2_lane = 0; i2_lane < road.numLanes; i2_lane++) {
                float offset2 = ((float)i2_lane - (float)road.numLanes / 2.0f) * road.width / road.numLanes;
                offset2 += road.width / (2 * road.numLanes);

                graphPoint point2;
                point2.position = sf::Vector2f(segment.p1_offset.x + (i - 1) * dx_s + offset2 * dx_a,
                                               segment.p1_offset.y + (i - 1) * dy_s + offset2 * dy_a);
                point2.angle = segment.angle;

                linkPoints(point, point2);
              }
            } else { // Or just the previous on the same lane
              graphPoint point2;
              point2.position = sf::Vector2f(segment.p1_offset.x + (i - 1) * dx_s + offset * dx_a,
                                             segment.p1_offset.y + (i - 1) * dy_s + offset * dy_a);
              point2.angle = segment.angle;

              linkPoints(point, point2);
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
        graphPoint point1;
        point1.angle = segment1.angle;
        point1.position = (distance(segment1.p1, intersection.center) < distance(segment1.p2, intersection.center))
                              ? segment1.p1_offset
                              : segment1.p2_offset;

        graphPoint point2;
        point2.angle = segment2.angle;
        point2.position = (distance(segment2.p1, intersection.center) < distance(segment2.p2, intersection.center))
                              ? segment2.p1_offset
                              : segment2.p2_offset;

        for (int iL_1 = 0; iL_1 < road1.numLanes; iL_1++) {
          float offset1 = ((float)iL_1 - (float)road1.numLanes / 2.0f) * road1.width / road1.numLanes;
          offset1 += road1.width / (2 * road1.numLanes);

          for (int iL_2 = 0; iL_2 < road2.numLanes; iL_2++) {
            float offset2 = ((float)iL_2 - (float)road2.numLanes / 2.0f) * road2.width / road2.numLanes;
            offset2 += road2.width / (2 * road2.numLanes);

            graphPoint point1_offset;
            point1_offset.angle = segment1.angle;
            point1_offset.position = sf::Vector2f(point1.position.x + offset1 * sin(segment1.angle),
                                                  point1.position.y + offset1 * -cos(segment1.angle));

            graphPoint point2_offset;
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
  ob::DubinsStateSpace space(TURNING_RADIUS);
  for (auto &point : graphPoints) {
    std::vector<graphPoint> newNeighbors;
    for (const auto &neighbor : neighbors[point]) {
      ob::State *start = space.allocState();
      ob::State *end = space.allocState();

      start->as<ob::DubinsStateSpace::StateType>()->setXY(point.position.x, point.position.y);
      start->as<ob::DubinsStateSpace::StateType>()->setYaw(point.angle);

      end->as<ob::DubinsStateSpace::StateType>()->setXY(neighbor.position.x, neighbor.position.y);
      end->as<ob::DubinsStateSpace::StateType>()->setYaw(neighbor.angle);

      float leftTurn = 0;
      float rightTurn = 0;

      // Extract the path
      ob::DubinsStateSpace::DubinsPath path = space.dubins(start, end);
      for (unsigned int i = 0; i < 3; ++i) // Dubins path has up to 3 segments
      {
        auto type = path.type_[i];
        double length = path.length_[i]; // Length of the segment

        if (type == ob::DubinsStateSpace::DubinsPathSegmentType::DUBINS_LEFT) {
          leftTurn += length;
        } else if (type == ob::DubinsStateSpace::DubinsPathSegmentType::DUBINS_RIGHT) {
          rightTurn += length;
        }
      }

      if (leftTurn < M_PI * 0.75 && rightTurn < M_PI * 0.75) {
        newNeighbors.push_back(neighbor);
      }
    }

    neighbors[point].clear();
    for (const auto &neighbor : newNeighbors) {
      neighbors[point].push_back(neighbor);
    }
  }
}

void CityGraph::linkPoints(const graphPoint &point, const graphPoint &neighbor) {
  std::vector<float> anglesPoint = {normalizeAngle(point.angle), normalizeAngle(point.angle + M_PI)};
  std::vector<float> anglesNeighbor = {normalizeAngle(neighbor.angle), normalizeAngle(neighbor.angle + M_PI)};

  graphPoint copyPoint = point;
  graphPoint copyNeighbor = neighbor;

  for (const auto &anglePoint : anglesPoint) {
    for (const auto &angleNeighbor : anglesNeighbor) {
      copyPoint.angle = anglePoint;
      copyNeighbor.angle = angleNeighbor;

      neighbors[copyPoint].push_back(copyNeighbor);
      neighbors[copyNeighbor].push_back(copyPoint);

      graphPoints.insert(copyPoint);
      graphPoints.insert(copyNeighbor);
    }
  }
}
