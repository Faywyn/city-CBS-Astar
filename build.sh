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
  rm -rf doc/latex
elif [ "$type" == "run" ]; then
  ./build/bin/city-cbs-astar $args
elif [ "$type" == "doc" ]; then
  echo "Warning: Create the latex files using doxygen first"
  if [ -d "doc/latex" ]; then
    cd doc/latex
    make
    cd ../.. 
    cp doc/latex/refman.pdf doc/documentation.pdf
  else
    echo "Error: The latex files are not created"
  fi
elif [ "$type" == "sign" ]; then
  codesign --entitlements Entitlements.plist -s - ./build/bin/city-cbs-astar
elif [ "$type" == "help" ]; then
  echo "Usage: ./build.sh [debug|release]: Build the project in debug or release mode"
  echo "Usage: ./build.sh [debugRun|releaseRun] [num_agents_min] [num_agents_max] [num_data]: Build and run the project in debug or release mode"
  echo "Usage: ./build.sh [clean]: Clean the build directory"
  echo "Usage: ./build.sh [run] [data] [num_agents_min] [num_agents_max] [num_data]"
  echo "Usage: ./build.sh [run] [run] [num_agents]"
  echo "Usage: ./build.sh [doc]: Create the documentation (doxygen and latex)"
  echo "Usage: ./build.sh [sign]: Sign the binary for MacOS"
else
  echo "Usage: ./build.sh [debug|debugRun|release|releaseRun|clean|doc|run|help]"
fi
