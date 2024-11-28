#include <iostream>
#include <ompl/base/State.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
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

    if (road.segments.size() == 1) {
      for (int i_lane = 0; i_lane < road.numLanes; i_lane++) {
        float offset = ((float)i_lane - (float)road.numLanes / 2.0f) * road.width / road.numLanes;
        offset += road.width / (2 * road.numLanes);

        graphPoint point1;
        point1.angle = road.segments[0].angle;
        point1.position = sf::Vector2f(road.segments[0].p1_offset.x + offset * sin(road.segments[0].angle),
                                       road.segments[0].p1_offset.y + offset * -cos(road.segments[0].angle));

        graphPoint point2;
        point2.angle = road.segments[0].angle;
        point2.position = sf::Vector2f(road.segments[0].p2_offset.x + offset * sin(road.segments[0].angle),
                                       road.segments[0].p2_offset.y + offset * -cos(road.segments[0].angle));

        neighbors[point1].push_back(point2);
        neighbors[point2].push_back(point1);
      }
      continue;
    }

    int numSeg = 0;
    for (const auto &segment : road.segments) {
      if (numSeg > 0) { // Link to the previous one
        std::cout << "Linking road segment " << numSeg - 1 << " to " << numSeg << " for road " << road.id << std::endl;
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

          neighbors[point1].push_back(point2);
          neighbors[point2].push_back(point1);

          graphPoints.insert(point1);
          graphPoints.insert(point2);
        }
      }
      numSeg++;

      float segmentLength =
          sqrt(pow(segment.p2_offset.x - segment.p1_offset.x, 2) + pow(segment.p2_offset.y - segment.p1_offset.y, 2));
      int numPoints = segmentLength / 10; // meters between each point
      float dx_s = (segment.p2_offset.x - segment.p1_offset.x) / numPoints;
      float dy_s = (segment.p2_offset.y - segment.p1_offset.y) / numPoints;
      float dx_a = sin(segment.angle);
      float dy_a = -cos(segment.angle);

      for (int i_lane = 0; i_lane < road.numLanes; i_lane++) {
        float offset = ((float)i_lane - (float)road.numLanes / 2.0f) * road.width / road.numLanes;
        offset += road.width / (2 * road.numLanes);

        for (int i = 0; i <= numPoints; i++) {
          graphPoint point;
          point.position = sf::Vector2f(segment.p1_offset.x + i * dx_s + offset * dx_a,
                                        segment.p1_offset.y + i * dy_s + offset * dy_a);
          point.angle = segment.angle;
          graphPoints.insert(point);

          bool isLast = i == numPoints;
          if (i == numPoints) {
            point.position = sf::Vector2f(segment.p2_offset.x + offset * dx_a, segment.p2_offset.y + offset * dy_a);
          }

          // Connect to the previous point (even if it's on another lane)
          if (i > 0) {
            if (i % 3 == 0 && !isLast) { // Connect to the previous on the other lane
              for (int i2_lane = 0; i2_lane < road.numLanes; i2_lane++) {
                float offset2 = ((float)i2_lane - (float)road.numLanes / 2.0f) * road.width / road.numLanes;
                offset2 += road.width / (2 * road.numLanes);

                graphPoint point2;
                point2.position = sf::Vector2f(segment.p1_offset.x + (i - 1) * dx_s + offset2 * dx_a,
                                               segment.p1_offset.y + (i - 1) * dy_s + offset2 * dy_a);
                point2.angle = segment.angle;
                graphPoints.insert(point2);

                neighbors[point].push_back(point2);
                neighbors[point2].push_back(point);
              }
            } else { // Or just the previous on the same lane
              graphPoint point2;
              point2.position = sf::Vector2f(segment.p1_offset.x + (i - 1) * dx_s + offset * dx_a,
                                             segment.p1_offset.y + (i - 1) * dy_s + offset * dy_a);
              point2.angle = segment.angle;
              graphPoints.insert(point2);

              neighbors[point].push_back(point2);
              neighbors[point2].push_back(point);
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

            neighbors[point1_offset].push_back(point2_offset);
            neighbors[point2_offset].push_back(point1_offset);
          }
        }
      }
    }
  }

  spdlog::info("Graph created with {} points", graphPoints.size());

  // Remove all the neighbors that need the Reed-Shepp curve to go backwards
  ob::ReedsSheppStateSpace reedsShepp(TURNING_RADIUS);
  for (auto &point : graphPoints) {
    std::vector<graphPoint> newNeighbors;
    for (const auto &neighbor : neighbors[point]) {
      // Check if they are to close and the angle is too big
      if (distance(point.position, neighbor.position) < 10 * CELL_SIZE &&
          std::abs(normalizeAngle(neighbor.angle - point.angle)) > M_PI / 8) {
        continue;
      }

      ob::State *start = reedsShepp.allocState();
      ob::State *end = reedsShepp.allocState();

      start->as<ob::ReedsSheppStateSpace::StateType>()->setXY(point.position.x, point.position.y);
      start->as<ob::ReedsSheppStateSpace::StateType>()->setYaw(point.angle);

      end->as<ob::ReedsSheppStateSpace::StateType>()->setXY(neighbor.position.x, neighbor.position.y);
      end->as<ob::ReedsSheppStateSpace::StateType>()->setYaw(neighbor.angle);

      ob::ReedsSheppStateSpace::ReedsSheppPath path = reedsShepp.reedsShepp(start, end);

      bool hasBackward = false;
      for (int i = 0; i < 5; ++i) { // Reeds-Shepp paths have up to 5 segments
        auto segmentType = path.type_[i];

        if (segmentType == ob::ReedsSheppStateSpace::RS_NOP) {
          break;
        }

        if (path.length_[i] < 0) {
          hasBackward = true;
          break;
        }
      }

      if (hasBackward) {
        std::cout << "The Reeds-Shepp path involves backward motion." << std::endl;
      } else {
        newNeighbors.push_back(neighbor);
      }
    }

    neighbors[point].clear();
    for (const auto &neighbor : newNeighbors) {
      neighbors[point].push_back(neighbor);
    }
  }
}
