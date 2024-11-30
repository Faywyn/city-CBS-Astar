#include "spdlog/spdlog.h"
#include <SFML/Graphics.hpp>

#include "cityMap.h"
#include "config.h"
#include "renderer.h"
#include "test.h"

int main() {
  spdlog::set_pattern("[%d-%m-%C %H:%M:%S.%e] [%^%l%$] [thread %t] %v");

  if (ENVIRONMENT == 0) {
    spdlog::set_level(spdlog::level::debug);

    // Test test;
    // test.runTests();
  } else {
    spdlog::set_level(spdlog::level::info);
  }

  CityMap cityMap;
  cityMap.loadFile("assets/map/map03.osm");

  CityGrid cityGrid;
  cityGrid.createGrid(cityMap);

  CityGraph cityGraph;
  cityGraph.createGraph(cityMap);

  Renderer renderer;
  renderer.startRender(cityMap, cityGrid, cityGraph);

  return 0;
}
