#include "spdlog/spdlog.h"
#include <SFML/Graphics.hpp>

#include "cityMap.h"
#include "config.h"
#include "fileSelector.h"
#include "manager.h"
#include "renderer.h"
#include "test.h"

int main(int nArgs, char **args) {
  spdlog::set_pattern("[%d-%m-%C %H:%M:%S.%e] [%^%l%$] [thread %t] %v");

  int numCars = 10;
  if (nArgs > 1) {
    numCars = std::stoi(args[1]);
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

  CityMap cityMap;
  cityMap.loadFile("assets/map/" + mapFile);

  CityGraph cityGraph;
  cityGraph.createGraph(cityMap);

  Manager manager(cityGraph, cityMap);
  if (nArgs > 2 && std::string(args[2]) == "cbs") {
    manager.createCarsCBS(numCars);
  } else {
    manager.createCarsAStar(numCars);
  }

  Renderer renderer;
  renderer.startRender(cityMap, cityGraph, manager);

  return 0;
}
