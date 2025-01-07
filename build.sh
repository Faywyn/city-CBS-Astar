#!/bin/bash
type="$1"
shift
args="$@"
if [ "$type" == "debug" ]; then
  cmake -DCMAKE_BUILD_TYPE=Debug -B build
  cmake --build build -j
elif [ "$type" == "debugRun" ]; then
  cmake -DCMAKE_BUILD_TYPE=Debug -B build
  cmake --build build -j
  ./build/bin/city-cbs-astar $args
elif [ "$type" == "release" ]; then
  cmake -DCMAKE_BUILD_TYPE=Release -B build
  cmake --build build -j
elif [ "$type" == "releaseRun" ]; then
  cmake -DCMAKE_BUILD_TYPE=Release -B build
  cmake --build build -j
  ./build/bin/city-cbs-astar $args
elif [ "$type" == "clean" ]; then
  rm -rf build
elif [ "$type" == "run" ]; then
  ./build/bin/city-cbs-astar $args
elif [ "$type" == "help" ]; then
  echo "Usage: ./build.sh [debug|release]: Build the project in debug or release mode"
  echo "Usage: ./build.sh [debugRun|releaseRun] [num_agents] [cbs]: Build and run the project in debug or release mode"
  echo "Usage: ./build.sh [clean]: Clean the build directory"
  echo "Usage: ./build.sh [run] [num_agents] [cbs]: Run the project, num_agents and cbs are optional (use cbs to enable CBS)"
else
  echo "Usage: ./build.sh [debug|debugRun|release|releaseRun|clean|run|help]"
fi
