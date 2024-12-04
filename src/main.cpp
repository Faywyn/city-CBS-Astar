#include "spdlog/spdlog.h"
#include <SFML/Graphics.hpp>

#include "cityMap.h"
#include "config.h"
#include "manager.h"
#include "renderer.h"
#include "test.h"

int main() {
  spdlog::set_pattern("[%d-%m-%C %H:%M:%S.%e] [%^%l%$] [thread %t] %v");

  if (ENVIRONMENT == 0) {
    spdlog::set_level(spdlog::level::debug);
    Test test;
    test.runTests();
  } else {
    spdlog::set_level(spdlog::level::info);
  }

  CityMap cityMap;
  cityMap.loadFile("assets/map/map01.osm");

  CityGraph cityGraph;
  cityGraph.createGraph(cityMap);

  Manager manager(cityGraph);
  manager.createCars(20);

  Renderer renderer;
  renderer.startRender(cityMap, cityGraph, manager);

  return 0;
}
