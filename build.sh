#!/bin/bash
# Legacy build.sh script - now calls the new manage.py script
# This file is kept for backward compatibility

type="$1"
shift
args="$@"

echo "Note: build.sh is deprecated. Please use 'python manage.py' or './manage.py' instead."
echo "See 'python manage.py help' for more information."
echo ""

if [ "$type" == "debug" ]; then
  python3 manage.py build --debug
elif [ "$type" == "debugRun" ]; then
  python3 manage.py build --debug && python3 manage.py run -- $args
elif [ "$type" == "release" ]; then
  python3 manage.py build --release
elif [ "$type" == "releaseRun" ]; then
  python3 manage.py build --release && python3 manage.py run -- $args
elif [ "$type" == "clean" ]; then
  python3 manage.py clean
elif [ "$type" == "run" ]; then
  python3 manage.py run -- $args
elif [ "$type" == "doc" ]; then
  python3 manage.py doc
elif [ "$type" == "sign" ]; then
  python3 manage.py sign
elif [ "$type" == "help" ]; then
  echo "Legacy build.sh commands:"
  echo "  ./build.sh debug         - Build in debug mode"
  echo "  ./build.sh release       - Build in release mode"
  echo "  ./build.sh debugRun ...  - Build in debug mode and run"
  echo "  ./build.sh releaseRun .. - Build in release mode and run"
  echo "  ./build.sh clean         - Clean build artifacts"
  echo "  ./build.sh run ...       - Run the executable"
  echo "  ./build.sh doc           - Generate documentation"
  echo "  ./build.sh sign          - Sign binary (macOS)"
  echo ""
  echo "New manage.py commands (recommended):"
  python3 manage.py help
else
  echo "Unknown command: $type"
  echo "Usage: ./build.sh [debug|debugRun|release|releaseRun|clean|doc|run|sign|help]"
  echo ""
  echo "Or use the new manage.py script:"
  python3 manage.py help
fi
