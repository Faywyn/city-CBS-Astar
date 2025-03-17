/**
 * @file main.cpp
 * @brief Main file
 *
 * This file contains the main function of the project. It is used to run the simulation and create data.
 */
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

  if (nArgs < 1) {
    spdlog::error("Usage: {} \"data\" [numCarsMin] [numCarsMax] [numData] || {} \"run\" [numCars]", args[0]);
    return 1;
  }

  bool data = args[1] == std::string("data");
  int runNumCars = 10;
  int dataNumCarsMin = 10;
  int dataNumCarsMax = 15;
  int dataNumData = -1;

  if (nArgs > 2) {
    runNumCars = std::stoi(args[2]);
    dataNumCarsMin = std::stoi(args[2]);
  }
  if (nArgs > 3) {
    dataNumCarsMax = std::stoi(args[3]);
  }
  if (nArgs > 4) {
    dataNumData = std::stoi(args[4]);
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

  if (data) {
    spdlog::info("Creating data for map {}, numData: {}, numCarsMin: {}, numCarsMax: {}", mapFile, dataNumData,
                 dataNumCarsMin, dataNumCarsMax);

    DataManager dataManager(mapFile);
    dataManager.createData(dataNumData, dataNumCarsMin, dataNumCarsMax, mapFile);
  } else {
    spdlog::info("Running simulation for map {}, numCars: {}", mapFile, runNumCars);

    CityMap cityMap;
    cityMap.loadFile("assets/map/" + mapFile);

    CityGraph cityGraph;
    cityGraph.createGraph(cityMap);

    Manager manager(cityGraph, cityMap, true);
    manager.createCarsCBS(runNumCars);

    Renderer renderer;
    renderer.startRender(cityMap, cityGraph, manager);
  }

  return 0;
}
