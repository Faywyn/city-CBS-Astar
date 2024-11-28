#include <iostream>
#include <ompl/base/State.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>

#include "cityGrid.h"
#include "config.h"

namespace ob = ompl::base;

void CityGrid::createGrid(const CityMap &cityMap) {
  numCellsX = cityMap.getWidth() / CELL_SIZE;
  numCellsY = cityMap.getHeight() / CELL_SIZE;
  numCells = numCellsX * numCellsY;
  grid.resize(numCells, false);

  for (const road &road : cityMap.getRoads()) {
    createGridForRoad(road);
  }
}

// Draw each lane using Reeds-Shepp curve (OMPL)
void CityGrid::createGridForRoad(const road &road) {
  std::vector<std::vector<std::pair<sf::Vector2f, float>>> lanes(road.numLanes); // Point and angle
  // First add each points to its lane
  for (auto s : road.segments) {
    float angle = s.angle;
    float dx = sin(angle);
    float dy = -cos(angle);

    for (int i = 0; i < road.numLanes; i++) {
      float offset = ((float)i - (float)road.numLanes / 2.0f) * road.width / road.numLanes;
      offset += road.width / (2 * road.numLanes);

      sf::Vector2f p1 = {s.p1.x + offset * dx, s.p1.y + offset * dy};
      sf::Vector2f p2 = {s.p2.x + offset * dx, s.p2.y + offset * dy};

      lanes[i].push_back({p1, angle});
      lanes[i].push_back({p2, angle});
    }
  }

  // Then draw the lanes
  ob::ReedsSheppStateSpace reedsShepp(TURNING_RADIUS);
  for (int i = 0; i < road.numLanes; i++) {
    for (int j = 0; j < (int)lanes[i].size() - 1; j++) {
      ob::State *start = reedsShepp.allocState();
      ob::State *end = reedsShepp.allocState();

      start->as<ob::ReedsSheppStateSpace::StateType>()->setXY(lanes[i][j].first.x, lanes[i][j].first.y);
      start->as<ob::ReedsSheppStateSpace::StateType>()->setYaw(lanes[i][j].second);

      end->as<ob::ReedsSheppStateSpace::StateType>()->setXY(lanes[i][j + 1].first.x, lanes[i][j + 1].first.y);
      end->as<ob::ReedsSheppStateSpace::StateType>()->setYaw(lanes[i][j + 1].second);

      // Draw the Reeds-Shepp curve
      // Depending on CELL_SIZE
      float step = CELL_SIZE / 2.0f;
      float distance = reedsShepp.distance(start, end);
      int numSteps = distance / step;
      for (int k = 0; k < numSteps; k++) {
        ob::State *state = reedsShepp.allocState();
        reedsShepp.interpolate(start, end, (float)k / (float)numSteps, state);

        float x = state->as<ob::ReedsSheppStateSpace::StateType>()->getX();
        float y = state->as<ob::ReedsSheppStateSpace::StateType>()->getY();

        int cellX = (int)(x / CELL_SIZE);
        int cellY = (int)(y / CELL_SIZE);

        if (cellX >= 0 && cellX < numCellsX && cellY >= 0 && cellY < numCellsY) {
          grid[cellY * numCellsX + cellX] = true;
        }

        reedsShepp.freeState(state);
      }
    }
  }
}
