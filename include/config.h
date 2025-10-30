/**
 * @file config.h
 * @brief Configuration file containing all project constants and parameters
 * 
 * This file centralizes all configuration parameters for the city-CBS-Astar project.
 * Modifying values here will affect the behavior of the entire application.
 */
#pragma once

#include <string>

// ============================================================================
// Environment Configuration
// ============================================================================
constexpr int ENVIRONMENT = 0; // 0 = development (debug logs, tests), 1 = production (info logs only)

// ============================================================================
// Display Configuration
// ============================================================================
constexpr int SCREEN_WIDTH = 2880;
constexpr int SCREEN_HEIGHT = 1864;
constexpr double LOG_CBS_REFRESHRATE = 0.3; // Refresh rate for CBS logging in seconds

// ============================================================================
// Map and Geographic Constants
// ============================================================================
constexpr int EARTH_RADIUS = 6371000; // Earth radius in meters for lat/lon conversions

// ============================================================================
// Road and Traffic Configuration
// ============================================================================
constexpr double DEFAULT_ROAD_WIDTH = 7.0;               // Default road width in meters
constexpr double DEFAULT_LANE_WIDTH = 3.5;               // Standard lane width in meters
constexpr double MIN_ROAD_WIDTH = 4.0;                   // Minimum acceptable road width in meters
constexpr bool ROAD_ENABLE_RIGHT_HAND_TRAFFIC = false;   // Enable right-hand traffic rules

// ============================================================================
// Path Planning Configuration
// ============================================================================
constexpr double DUBINS_INTERPOLATION_STEP = 0.1;        // Dubins curve interpolation step in meters

// ============================================================================
// Visualization Controls
// ============================================================================
constexpr double ZOOM_SPEED = 0.1;                       // Camera zoom speed multiplier
constexpr double MOVE_SPEED = 0.01;                      // Camera movement speed multiplier

// ============================================================================
// Simulation Parameters
// ============================================================================
constexpr double SIM_STEP_TIME = 0.05;                   // Simulation time step in seconds
constexpr int CBS_PRECISION_FACTOR = 1;                  // CBS precision factor (CBS_PRECISION_FACTOR * SIM_STEP_TIME should be reasonable)
constexpr double CBS_MAX_SUB_TIME = 30;                  // Maximum sub-problem solving time in seconds
constexpr double CBS_MAX_OPENSET_SIZE = 5;               // Maximum size of CBS open set

constexpr double OCBS_CONFLICT_RANGE = SIM_STEP_TIME * 5; // Conflict detection range for OCBS algorithm

// ============================================================================
// Discretization Parameters (for hash functions and state space)
// ============================================================================
constexpr double CELL_SIZE = 0.1;                        // Spatial discretization in meters
constexpr double SPEED_RESOLUTION = 0.3;                 // Speed discretization in m/s
constexpr double ANGLE_RESOLUTION = 0.1;                 // Angular discretization in radians
constexpr double TIME_RESOLUTION = SIM_STEP_TIME;        // Temporal discretization in seconds

// ============================================================================
// Vehicle Properties
// ============================================================================
constexpr double CAR_MIN_TURNING_RADIUS = 1.5;          // Minimum turning radius in meters
constexpr double CAR_MAX_SPEED_KM = 30.0;               // Maximum speed in km/h
constexpr double CAR_MAX_SPEED_MS = CAR_MAX_SPEED_KM / 3.6; // Maximum speed in m/s
constexpr double CAR_MAX_G_FORCE = 0.5;                 // Maximum lateral acceleration in m/s^2
constexpr double CAR_ACCELERATION = 1;                  // Forward acceleration in m/s^2
constexpr double CAR_DECELERATION = 1;                  // Braking deceleration in m/s^2
constexpr double CAR_LENGTH = 4.2;                      // Vehicle length in meters
constexpr double CAR_WIDTH = 1.6;                       // Vehicle width in meters

// ============================================================================
// Algorithm Parameters
// ============================================================================
constexpr double COLLISION_SAFETY_FACTOR = 1.1;         // Safety margin multiplier for collision detection
constexpr int ASTAR_MAX_ITERATIONS = 100000;            // Maximum iterations for A* pathfinding
constexpr int NUM_SPEED_DIVISIONS = 5;                  // Number of speed divisions for trajectory planning
constexpr double GRAPH_POINT_DISTANCE = 15.0;           // Distance between graph nodes in meters
