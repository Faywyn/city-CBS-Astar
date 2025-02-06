
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

#include <spdlog/spdlog.h>

#include "cityGraph.h"
#include "cityMap.h"
#include "dataManager.h"
#include "manager.h"

DataManager::DataManager(std::string filename) {
  // Create /data folder if it doesn't exist
  if (!std::filesystem::exists("data")) {
    spdlog::debug("Creating data folder");
    std::filesystem::create_directory("data");
  }
}

void DataManager::createData(int numData, int numCars, std::string mapName) {

  std::string mapNameNoExt = mapName.substr(0, mapName.find_last_of("."));
  std::string filename = "data/" + mapNameNoExt + "_" + std::to_string(numCars) + "_data.csv";

  CityMap cityMap;
  cityMap.loadFile("assets/map/" + mapName);

  CityGraph cityGraph;
  cityGraph.createGraph(cityMap);

  std::ofstream file;
  file.open(filename, std::ios::app);

  for (int i = 0; i < numData; i++) {
    Manager manager(cityGraph, cityMap, false);
    data resData = manager.createCarsCBS(numCars);

    file << resData.numCars << ";" << resData.carDensity << ";" << resData.carAvgSpeed << ";" << resData.carMaxSpeed
         << ";" << resData.carMinSpeed << std::endl;

    spdlog::info("Data {}/{} created: numCars: {}, carDensity: {}, carAvgSpeed: {}, carMaxSpeed: {}, carMinSpeed: {}",
                 i + 1, numData, resData.numCars, resData.carDensity, resData.carAvgSpeed, resData.carMaxSpeed,
                 resData.carMinSpeed);
  }

  file.close();
}
