#pragma once

#include <string>

constexpr int ENVIRONMENT = 0; // 0 = development, 1 = production
constexpr int SCREEN_WIDTH = 1920 / 2;
constexpr int SCREEN_HEIGHT = 1080 / 2;

constexpr int EARTH_RADIUS = 6371000; // in meters

constexpr float DEFAULT_ROAD_WIDTH = 7.0f; // in meters
constexpr float DEFAULT_LANE_WIDTH = 3.5f; // in meters
constexpr float MIN_ROAD_WIDTH = 4.0f;     // in meters

constexpr float ZOOM_SPEED = 0.1f;
constexpr float MOVE_SPEED = 0.01f;

constexpr float SIM_STEP_TIME = 0.02f; // in seconds
constexpr float CELL_SIZE = 0.5f;      // in meters

constexpr float CAR_MIN_TURNING_RADIUS = 4.0f;              // in meters
constexpr float CAR_MAX_SPEED_KM = 50.0f;                   // in km/h
constexpr float CAR_MAX_SPEED_MS = CAR_MAX_SPEED_KM / 3.6f; // in m/s
constexpr float CAR_MAX_G_FORCE = 9.0f;                     // in m/s^2
constexpr float CAR_ACCELERATION = 0.1f;                    // in m/s^2
constexpr float CAR_LENGTH = 4.2f;                          // in meters
constexpr float CAR_WIDTH = 1.6f;                           // in meters
