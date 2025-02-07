#include "spdlog/spdlog.h"
#include <SFML/Graphics.hpp>

#include "cityMap.h"
#include "config.h"
#include "dataManager.h"
#include "fileSelector.h"
#include "manager.h"
#include "renderer.h"
#include "test.h"

int main(int nArgs, char **args) {
  srand(time(NULL));
  spdlog::set_pattern("[%d-%m-%C %H:%M:%S.%e] [%^%l%$] [thread %t] %v");

  int numCarsMin = 10;
  int numCarsMax = 15;
  int numData = -1;
  if (nArgs > 1) {
    numCarsMin = std::stoi(args[1]);
  }
  if (nArgs > 2) {
    numCarsMax = std::stoi(args[2]);
  }
  if (nArgs > 3) {
    numData = std::stoi(args[3]);
  }

  FileSelector fileSelector("assets/map");
  std::string mapFile = fileSelector.selectFile();
  // std::string mapFile = "small01.osm";

  if (ENVIRONMENT == 0 && false) {
    spdlog::set_level(spdlog::level::debug);
    Test test;
    test.runTests();
  } else {
    spdlog::set_level(spdlog::level::info);
  }

  DataManager dataManager(mapFile);
  dataManager.createData(numData, numCarsMin, numCarsMax, mapFile);

  return 0;
}
