/**
 * @file dataManager.h
 * @brief Data manager
 *
 * This file contains the data manager class.
 */
#pragma once

#include <string>
#include <vector>

/**
 * @struct _data
 * @brief Data structure
 *
 * This struct represents the data structure.
 */
struct _data {
  double numCars;
  double carDensity;
  std::vector<double> carAvgSpeed;
};

/**
 * @class DataManager
 * @brief Data manager
 *
 * This class represents the data manager. It creates data and stores it in a file.
 */
class DataManager {
public:
  using data = _data;

  /**
   * @brief Constructor
   * @param filename The filename
   */
  DataManager(std::string filename);

  /**
   * @brief Create data. It launches multiple simulations with different number of cars and car densities.
   * Then, it calculates different statistics and stores them in a file.
   * @param numData The number of data
   * @param numCarsMin The minimum number of cars
   * @param numCarsMax The maximum number of cars
   * @param mapName The map name
   */
  void createData(int numData, int numCarsMin, int numCarsMax, std::string mapName);

private:
};
