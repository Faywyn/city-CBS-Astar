#!/bin/bash
if [ "$1" == "debug" ]; then
  cmake -DCMAKE_BUILD_TYPE=Debug -B build
  cmake --build build -j
elif [ "$1" == "debugRun" ]; then
  cmake -DCMAKE_BUILD_TYPE=Debug -B build
  cmake --build build -j
  ./build/bin/city-cbs-astar
elif [ "$1" == "release" ]; then
  cmake -DCMAKE_BUILD_TYPE=Release -B build
  cmake --build build -j
elif [ "$1" == "releaseRun" ]; then
  cmake -DCMAKE_BUILD_TYPE=Release -B build
  cmake --build build -j
  ./build/bin/city-cbs-astar
elif [ "$1" == "clean" ]; then
  rm -rf build
elif [ "$1" == "run" ]; then
  ./build/bin/city-cbs-astar
else
  echo "Usage: ./build.sh [debug|debugRun|release|releaseRun|clean|run]"
fi
