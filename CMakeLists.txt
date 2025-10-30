cmake_minimum_required(VERSION 3.16)
project(city-cbs-astar LANGUAGES CXX)

# Set C++ standard to C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Note: On Windows with vcpkg, use: -DCMAKE_TOOLCHAIN_FILE=path/to/vcpkg/scripts/buildsystems/vcpkg.cmake

# Ensure arm64 on macOS (only if on macOS and not already set)
if(APPLE AND NOT CMAKE_OSX_ARCHITECTURES)
  set(CMAKE_OSX_ARCHITECTURES "arm64" CACHE STRING "Build architecture" FORCE)
endif()

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

# Compiler flags for different platforms and build types
if(MSVC)
  # MSVC (Windows)
  set(CMAKE_CXX_FLAGS_DEBUG "/W4 /Zi /Od /MDd")
  set(CMAKE_CXX_FLAGS_RELEASE "/O2 /DNDEBUG /MD")
else()
  # GCC/Clang (Linux/macOS)
  set(CMAKE_CXX_FLAGS_DEBUG "-g -Wall -Wextra")
  # Only add sanitizers if supported (not all platforms support them)
  if(NOT WIN32)
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fsanitize=address")
    set(CMAKE_LINKER_FLAGS_DEBUG "${CMAKE_LINKER_FLAGS_DEBUG} -fsanitize=address")
  endif()
  set(CMAKE_CXX_FLAGS_RELEASE "-O3 -ffast-math -DNDEBUG")
endif()

# Set policy for FindBoost module (CMake 3.30+)
if(POLICY CMP0167)
  cmake_policy(SET CMP0167 NEW)
endif()

# Find Boost with better error handling
find_package(Boost QUIET)
if(Boost_FOUND)
    message(STATUS "Boost found: ${Boost_INCLUDE_DIRS}")
    include_directories(${Boost_INCLUDE_DIRS})
else()
    message(WARNING "Boost not found! Install it with: sudo apt-get install libboost-all-dev (Linux) or brew install boost (macOS)")
    message(FATAL_ERROR "Boost is required to build this project")
endif()

# Find OMPL with better error handling
find_package(ompl QUIET)
if(OMPL_FOUND OR ompl_FOUND)
    message(STATUS "OMPL found: ${OMPL_INCLUDE_DIRS}")
    include_directories(${OMPL_INCLUDE_DIRS})
else()
    message(WARNING "OMPL not found! Install it with: sudo apt-get install libompl-dev (Linux) or brew install ompl (macOS)")
    message(FATAL_ERROR "OMPL is required to build this project")
endif()

# Fetch external dependencies using FetchContent
include(FetchContent)

FetchContent_Declare(SFML
    GIT_REPOSITORY https://github.com/SFML/SFML.git
    GIT_TAG 3.0.x)
FetchContent_MakeAvailable(SFML)

FetchContent_Declare(SPDLOG
    GIT_REPOSITORY https://github.com/gabime/spdlog.git
    GIT_TAG v1.15.3
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
  src/dataManager.cpp
  src/fileSelector.cpp
  src/main.cpp 
  src/renderer.cpp
  src/test.cpp
  src/utils.cpp
  src/managers/index.cpp
  src/managers/ocbs.cpp
  src/dubins/interpolator.cpp
)

# Include directories
target_include_directories(${PROJECT_NAME} PUBLIC
  include
)

# Link libraries
target_link_libraries(${PROJECT_NAME} PUBLIC
  sfml-graphics
  spdlog::spdlog
  tinyxml2
  ${OMPL_LIBRARIES}
)

# Ensure C++17 standard is used
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
