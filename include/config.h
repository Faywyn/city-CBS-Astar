/**
 * @file config.h
 * @brief Configuration file
 */
#pragma once

#include <string>

constexpr int ENVIRONMENT = 0; // 0 = development, 1 = production
constexpr int SCREEN_WIDTH = 2880;
constexpr int SCREEN_HEIGHT = 1864;
constexpr double LOG_CBS_REFRESHRATE = 0.3; // in seconds

constexpr int EARTH_RADIUS = 6371000; // in meters

constexpr double DEFAULT_ROAD_WIDTH = 7.0; // in meters
constexpr double DEFAULT_LANE_WIDTH = 3.5; // in meters
constexpr double MIN_ROAD_WIDTH = 4.0;     // in meters
constexpr bool ROAD_ENABLE_RIGHT_HAND_TRAFFIC = false;

constexpr double ZOOM_SPEED = 0.1;
constexpr double MOVE_SPEED = 0.01;

constexpr double SIM_STEP_TIME = 0.05;  // in seconds
constexpr int CBS_PRECISION_FACTOR = 1; // CBS_PRECISION_FACTOR * SIM_STEP_TIME must not be to high
constexpr double CBS_MAX_SUB_TIME = 30; // in seconds
constexpr double CBS_MAX_OPENSET_SIZE = 5;

// For hash functions (to reduce items that are really close to each other)
constexpr double CELL_SIZE = 0.5;                 // in meters
constexpr double SPEED_RESOLUTION = 0.3;          // in m/s
constexpr double ANGLE_RESOLUTION = 0.1;          // in radians
constexpr double TIME_RESOLUTION = SIM_STEP_TIME; // in seconds

constexpr double CAR_MIN_TURNING_RADIUS = 1.5;              // in meters
constexpr double CAR_MAX_SPEED_KM = 30.0;                   // in km/h
constexpr double CAR_MAX_SPEED_MS = CAR_MAX_SPEED_KM / 3.6; // in m/s
constexpr double CAR_MAX_G_FORCE = 5.0;                     // in m/s^2
constexpr double CAR_ACCELERATION = 3.0;                    // in m/s^2
constexpr double CAR_DECELERATION = 4.0;                    // in m/s^2
constexpr double CAR_LENGTH = 4.2;                          // in meters
constexpr double CAR_WIDTH = 1.6;                           // in meters
