#pragma once

#include <string>

struct _data {
  double numCars;
  double carDensity;
  double carAvgSpeed;
  double carMaxSpeed;
  double carMinSpeed;
};

class DataManager {
public:
  using data = _data;

  DataManager(std::string filename);

  void createData(int numData, int numCars, std::string mapName);

private:
};
