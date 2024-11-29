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

constexpr float CELL_SIZE = 0.5f; // in meters

constexpr float TURNING_RADIUS = 3.0f; // in meters
