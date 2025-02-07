#pragma once

#include <string>
#include <vector>

struct _data {
  double numCars;
  double carDensity;
  std::vector<double> carAvgSpeed;
};

class DataManager {
public:
  using data = _data;

  DataManager(std::string filename);

  void createData(int numData, int numCarsMin, int numCarsMax, std::string mapName);

private:
};
