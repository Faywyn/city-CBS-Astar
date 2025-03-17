cmake_minimum_required(VERSION 3.16)
project(city-cbs-astar LANGUAGES CXX)

# Set C++ standard to C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Ensure arm64
set(CMAKE_OSX_ARCHITECTURES "arm64")

# Ensure compile commands are generated
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Output directory for binaries
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# Option for shared libraries (if needed)
option(BUILD_SHARED_LIBS "Build shared libraries" OFF)

# Default to Release if no build type is set
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Debug flags: Include debugging symbols, profiling tools, and sanitizers for memory checks
set(CMAKE_CXX_FLAGS_DEBUG "-g -Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} -fsanitize=address -g")
set(CMAKE_LINKER_FLAGS_DEBUG "${CMAKE_LINKER_FLAGS} -fsanitize=address")

# Release flags: Optimize for speed, disable debugging symbols, and prioritize performance
set(CMAKE_CXX_FLAGS_RELEASE "-Ofast -DNDEBUG")

find_package(Boost REQUIRED)

if(Boost_FOUND)
    message(STATUS "Boost found: ${Boost_INCLUDE_DIRS}")
    include_directories(${Boost_INCLUDE_DIRS})
else()
    message(FATAL_ERROR "Boost not found!")
endif()

find_package(ompl REQUIRED)

if(OMPL_FOUND)
    message(STATUS "OMPL found: ${OMPL_INCLUDE_DIRS}")
    include_directories(${OMPL_INCLUDE_DIRS})
else()
    message(FATAL_ERROR "OMPL not found!")
endif()

# Fetch external dependencies using FetchContent
include(FetchContent)

FetchContent_Declare(SFML
    GIT_REPOSITORY https://github.com/SFML/SFML.git
    GIT_TAG 2.6.x)
FetchContent_MakeAvailable(SFML)

FetchContent_Declare(SPDLOG
    GIT_REPOSITORY https://github.com/gabime/spdlog.git
    GIT_TAG v1.14.1
)
FetchContent_MakeAvailable(SPDLOG)

FetchContent_Declare(tinyxml2
    GIT_REPOSITORY https://github.com/leethomason/tinyxml2.git
    GIT_TAG 10.0.0
)
FetchContent_MakeAvailable(tinyxml2)

# Link directories
link_directories(${OMPL_LIBRARY_DIRS})

add_executable(${PROJECT_NAME}
  src/aStar.cpp
  src/car.cpp
  src/cityGraph.cpp
  src/cityMap.cpp
  src/constraintController.cpp
  src/dataManager.cpp
  src/dubins.cpp
  src/fileSelector.cpp
  src/main.cpp 
  src/manager.cpp
  src/managerCBS.cpp
  src/renderer.cpp
  src/test.cpp
  src/timedAStar.cpp
  src/utils.cpp
)

# Include directories
target_include_directories(${PROJECT_NAME} PRIVATE
  include
)

# Link libraries
target_link_libraries(${PROJECT_NAME} PRIVATE
  sfml-graphics
  spdlog::spdlog
  tinyxml2
  ${OMPL_LIBRARIES}
)

# Ensure C++17 standard is used
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_17)
