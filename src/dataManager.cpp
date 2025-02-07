#include <chrono>
#include <climits>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <mutex>
#include <random>
#include <string>
#include <thread>
#include <vector>

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

// void DataManager::createData(int numData, int numCarsMin, int numCarsMax, std::string mapName) {
//   numData = numData < 1 ? INT_MAX : numData;
//
//   std::string mapNameNoExt = mapName.substr(0, mapName.find_last_of("."));
//   std::string filename = "data/" + mapNameNoExt + "_data.csv";
//
//   CityMap cityMap;
//   cityMap.loadFile("assets/map/" + mapName);
//
//   CityGraph cityGraph;
//   cityGraph.createGraph(cityMap);
//
//   std::ofstream file;
//   file.open(filename, std::ios::app);
//
//   for (int i = 0; i < numData; i++) {
//     int numCars = rand() % (numCarsMax - numCarsMin + 1) + numCarsMin;
//
//     Manager manager(cityGraph, cityMap, false);
//     data resData = manager.createCarsCBS(numCars);
//
//     file << resData.numCars << ";" << resData.carDensity << ";" << resData.carAvgSpeed << ";" << resData.carMaxSpeed
//          << ";" << resData.carMinSpeed << std::endl;
//
//     spdlog::info("Data {}/{} created: numCars: {}, carDensity: {}, carAvgSpeed: {}, carMaxSpeed: {}, carMinSpeed:
//     {}",
//                  i + 1, numData, resData.numCars, resData.carDensity, resData.carAvgSpeed, resData.carMaxSpeed,
//                  resData.carMinSpeed);
//   }
//
//   file.close();
// }

void DataManager::createData(int numData, int numCarsMin, int numCarsMax, std::string mapName) {
  int numThread = NUM_THREADS;

  // If numData is less than 1, default to a very high number (as in your original code).
  numData = numData < 1 ? INT_MAX : numData;

  // Remove file extension from mapName to construct the output filename.
  std::string mapNameNoExt = mapName.substr(0, mapName.find_last_of("."));
  std::string filename = "data/" + mapNameNoExt + "_" + std::to_string((int)CBS_MAX_SUB_TIME) + "_data.csv";

  // Load the city map.
  CityMap cityMap;
  cityMap.loadFile("assets/map/" + mapName);

  // Create the city graph.
  CityGraph cityGraph;
  cityGraph.createGraph(cityMap);

  // Open the output file in append mode.
  std::ofstream file;
  file.open(filename, std::ios::app);
  if (!file.is_open()) {
    spdlog::error("Failed to open file {}", filename);
    return;
  }

  // Use multi threading to create the data.
  std::mutex fileMutex;
  std::vector<std::thread> threads;
  threads.reserve(numThread);

  for (int threadId = 0; threadId < numThread; ++threadId) {
    threads.emplace_back([=, &cityGraph, &cityMap, &file, &fileMutex]() {
      std::mt19937 rng(std::chrono::steady_clock::now().time_since_epoch().count() + threadId);
      std::uniform_int_distribution<int> dist(numCarsMin, numCarsMax);

      for (int i = threadId; i < numData; i += numThread) {
        int numCars = dist(rng);

        Manager manager(cityGraph, cityMap, false);
        data resData = manager.createCarsCBS(numCars);

        {
          std::lock_guard<std::mutex> lock(fileMutex);
          file << resData.numCars << ";" << resData.carDensity;
          for (auto speed : resData.carAvgSpeed) {
            file << ";" << speed;
          }
          file << std::endl;
        }

        if (numData == INT_MAX) {
          spdlog::info("Data {}: numCars: {}, carDensity: {:0>6.5}", i + 1, resData.numCars, resData.carDensity);
        } else {
          spdlog::info("Data {}: numCars: {}, carDensity: {:0>6.5}", i + 1, numData, resData.numCars,
                       resData.carDensity);
        }
      }
    });
  }

  // Wait for all threads to finish.
  for (auto &t : threads) {
    t.join();
  }

  file.close();
}
