#pragma once

#include "cityMap.h"

class CityGrid {
public:
  void createGrid(const CityMap &cityMap);

  // Getters
  std::vector<bool> getGrid() const { return grid; }
  int getNumCells() const { return numCells; }
  int getNumCellsX() const { return numCellsX; }
  int getNumCellsY() const { return numCellsY; }

private:
  void createGridForRoad(const road &road);
  void createGridForIntersection(const intersection &intersection);

  int numCells;
  int numCellsX;
  int numCellsY;
  std::vector<bool> grid;
};
