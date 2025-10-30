/**
 * @file main.cpp
 * @brief Main file
 *
 * This file contains the main function of the project. It is used to run the simulation and create data.
 */
#include "cityMap.h"
#include "config.h"
#include "dataManager.h"
#include "fileSelector.h"
#include "manager.h"
#include "manager_ocbs.h"
#include "renderer.h"
#include "spdlog/spdlog.h"
#include "test.h"
#include <SFML/Graphics.hpp>

int main(int nArgs, char **args) {
  // Initialize random seed for reproducibility
  srand(time(NULL));
  
  // Configure logging format with timestamp, log level, and thread info
  spdlog::set_pattern("[%d-%m-%C %H:%M:%S.%e] [%^%l%$] [thread %t] %v");

  if (nArgs < 1) {
    spdlog::error("Usage: {} \"data\" [numCarsMin] [numCarsMax] [numData] || {} \"run\" [numCars]", args[0]);
    return 1;
  }

  // Parse command line arguments
  bool data = args[1] == std::string("data");
  
  // Default values for simulation parameters
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

  // Select the map file to use from the assets directory
  FileSelector fileSelector("assets/map");
  std::string mapFile = fileSelector.selectFile();

  // Set logging level based on environment (development vs production)
  if (ENVIRONMENT == 0) {
    spdlog::set_level(spdlog::level::debug);
    // Run tests in development mode to ensure dependencies are working
    Test test;
    test.runTests();
  } else {
    spdlog::set_level(spdlog::level::info);
  }

  // Execute the appropriate mode: data generation or simulation
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

    ManagerCBS manager(cityGraph, cityMap);
    manager.initializeAgents(runNumCars);

    Renderer renderer;
    renderer.startRender(cityMap, cityGraph, manager);
  }

  return 0;
}
