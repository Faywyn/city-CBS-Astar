/**
 * @file dataManager.cpp
 * @brief Data manager
 *
 * This file contains the implementation of the DataManager class.
 */
#include <filesystem>
#include <fstream>
#include <iostream>
#include <random>
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

void DataManager::createData(int numData, int numCarsMin, int numCarsMax, std::string mapName) {

  spdlog::error("Deprecated: Use the new data manager class");

  return;

  // // If numData is less than 1, default to a very high number (as in your original code).
  // numData = numData < 1 ? INT_MAX : numData;
  //
  // // Remove file extension from mapName to construct the output filename.
  // std::string mapNameNoExt = mapName.substr(0, mapName.find_last_of("."));
  // std::string filename = "data/" + mapNameNoExt + "_" + std::to_string((int)CBS_MAX_SUB_TIME) +
  //                        (ROAD_ENABLE_RIGHT_HAND_TRAFFIC ? "_RHT" : "") + "_data.csv";
  //
  // // Load the city map.
  // CityMap cityMap;
  // cityMap.loadFile("assets/map/" + mapName);
  //
  // // Create the city graph.
  // CityGraph cityGraph;
  // cityGraph.createGraph(cityMap);
  //
  // // Open the output file in append mode.
  // std::ofstream file;
  // file.open(filename, std::ios::app);
  // if (!file.is_open()) {
  //   spdlog::error("Failed to open file {}", filename);
  //   return;
  // }
  //
  // std::mt19937 rng(std::chrono::steady_clock::now().time_since_epoch().count());
  // std::uniform_int_distribution<int> dist(numCarsMin, numCarsMax);
  //
  // for (int i = 0; i < numData; i += 1) {
  //   int numCars = dist(rng);
  //
  //   Manager manager(cityGraph, cityMap, false);
  //   auto resData = manager.createCarsCBS(numCars);
  //   if (!resData.first) {
  //     spdlog::warn("Data {}: CBS failed (numCars: {})", i + 1, numCars);
  //     i--;
  //     continue;
  //   }
  //
  //   data validResData = resData.second;
  //
  //   file << validResData.numCars << ";" << validResData.carDensity;
  //   for (auto speed : validResData.carAvgSpeed) {
  //     file << ";" << speed;
  //   }
  //   file << std::endl;
  //
  //   if (numData == INT_MAX) {
  //     spdlog::info("Data {}: numCars: {}, carDensity: {:0>6.5}", i + 1, validResData.numCars,
  //     validResData.carDensity);
  //   } else {
  //     spdlog::info("Data {}: numCars: {}, carDensity: {:0>6.5}", i + 1, numData, validResData.numCars,
  //                  validResData.carDensity);
  //   }
  // }
  //
  // file.close();
}
