#pragma once

#include <string>

constexpr int ENVIRONMENT = 0; // 0 = development, 1 = production
constexpr int SCREEN_WIDTH = 1920 / 2;
constexpr int SCREEN_HEIGHT = 1080 / 2;

constexpr int EARTH_RADIUS = 6371000; // in meters

constexpr double DEFAULT_ROAD_WIDTH = 7.0f; // in meters
constexpr double DEFAULT_LANE_WIDTH = 3.5f; // in meters
constexpr double MIN_ROAD_WIDTH = 4.0f;     // in meters

constexpr double ZOOM_SPEED = 0.1f;
constexpr double MOVE_SPEED = 0.01f;

constexpr double SIM_STEP_TIME = 0.03f; // in seconds

// For hash functions (to reduce items that are really close to each other -> so equal)
constexpr double CELL_SIZE = 1.0f;        // in meters
constexpr double SPEED_RESOLUTION = 0.5f; // in m/s
constexpr double ANGLE_RESOLUTION = 0.1f; // in radians
constexpr double TIME_RESOLUTION = 0.1f;  // in seconds

constexpr double CAR_MIN_TURNING_RADIUS = 4.0f;              // in meters
constexpr double CAR_MAX_SPEED_KM = 50.0f;                   // in km/h
constexpr double CAR_MAX_SPEED_MS = CAR_MAX_SPEED_KM / 3.6f; // in m/s
constexpr double CAR_MAX_G_FORCE = 9.0f;                     // in m/s^2
constexpr double CAR_ACCELERATION = 1.0f;                    // in m/s^2
constexpr double CAR_DECELERATION = 1.0f;                    // in m/s^2
constexpr double CAR_LENGTH = 4.2f;                          // in meters
constexpr double CAR_WIDTH = 1.6f;                           // in meters
constexpr double CAR_CBS_MIN_SPACING = 2.0f;                 // in meters
constexpr double CAR_CBS_TIME_GAP = 3 * TIME_RESOLUTION;     // in seconds
