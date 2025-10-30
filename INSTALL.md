# Installation Guide

This document provides detailed instructions for installing dependencies and building the city-CBS-Astar project on different platforms.

## Table of Contents
1. [Prerequisites](#prerequisites)
2. [Linux Installation](#linux-installation)
3. [macOS Installation](#macos-installation)
4. [Windows Installation](#windows-installation)
5. [Building the Project](#building-the-project)
6. [Troubleshooting](#troubleshooting)

## Prerequisites

All platforms require:
- **CMake** (version 3.16 or higher)
- **C++17** compatible compiler
- **Python 3** (for the build management script)
- **Boost** library
- **OMPL** (Open Motion Planning Library)

The following dependencies are automatically fetched by CMake:
- **SFML 2** (via FetchContent)
- **spdlog** (via FetchContent)
- **tinyxml2** (via FetchContent)

## Linux Installation

### Ubuntu/Debian

```bash
# Update package list
sudo apt-get update

# Install build tools
sudo apt-get install -y build-essential cmake ninja-build python3

# Install dependencies
sudo apt-get install -y libboost-all-dev libompl-dev

# For documentation (optional)
sudo apt-get install -y doxygen graphviz
```

### Fedora/RHEL

```bash
# Install build tools
sudo dnf install -y gcc-c++ cmake ninja-build python3

# Install dependencies
sudo dnf install -y boost-devel ompl-devel

# For documentation (optional)
sudo dnf install -y doxygen graphviz
```

### Arch Linux

```bash
# Install build tools and dependencies
sudo pacman -S base-devel cmake ninja python boost ompl

# For documentation (optional)
sudo pacman -S doxygen graphviz
```

## macOS Installation

### Using Homebrew (Recommended)

```bash
# Install Homebrew if not already installed
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"

# Install dependencies
brew install cmake boost ompl ninja python3

# For documentation (optional)
brew install doxygen graphviz
```

### Notes for macOS

- The project is configured to build for arm64 architecture by default on Apple Silicon Macs
- If you're on an Intel Mac, the CMake configuration will automatically adjust
- Code signing is available via `python manage.py sign` (requires macOS)

## Windows Installation

### Option 1: Using vcpkg (Recommended)

```powershell
# Install vcpkg
git clone https://github.com/microsoft/vcpkg.git
cd vcpkg
.\bootstrap-vcpkg.bat

# Install dependencies
.\vcpkg install boost ompl

# Add vcpkg toolchain to your environment (or specify in CMake)
$env:CMAKE_TOOLCHAIN_FILE = "$PWD\scripts\buildsystems\vcpkg.cmake"
```

### Option 2: Manual Installation

1. **Install Visual Studio 2019 or later** with C++ development tools
2. **Install CMake** from https://cmake.org/download/
3. **Install Python 3** from https://www.python.org/downloads/
4. **Install Ninja** (optional but recommended): `choco install ninja`
5. **Download and build Boost** from https://www.boost.org/
6. **Download and build OMPL** from https://ompl.kavrakilab.org/

### Notes for Windows

- Use Visual Studio Developer Command Prompt or PowerShell with MSVC environment loaded
- The build system uses Ninja generator on Windows for faster builds
- Sanitizers are not available on Windows builds

## Building the Project

Once dependencies are installed, building is the same on all platforms:

### Using the Python script (Recommended)

```bash
# Build in release mode (optimized)
python manage.py build --release

# Build in debug mode (with debugging symbols)
python manage.py build --debug

# Build with specific number of parallel jobs
python manage.py build --release -j 4
```

### Using the legacy build.sh (Linux/macOS only)

```bash
# Make executable (first time only)
chmod +x build.sh

# Build in release mode
./build.sh release

# Build in debug mode
./build.sh debug
```

### Using CMake directly

```bash
# Configure
cmake -DCMAKE_BUILD_TYPE=Release -B build

# Build
cmake --build build -j

# On Windows with vcpkg
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE=path/to/vcpkg/scripts/buildsystems/vcpkg.cmake -B build
cmake --build build -j
```

## Running the Project

After building, run the executable:

```bash
# Using the Python script
python manage.py run -- data 1 10 5
python manage.py run -- run 5

# Using the legacy build.sh (Linux/macOS)
./build.sh run data 1 10 5
./build.sh run run 5

# Direct execution
./build/bin/city-cbs-astar data 1 10 5
./build/bin/city-cbs-astar run 5
```

## Other Commands

```bash
# Clean build artifacts
python manage.py clean

# Generate documentation (requires Doxygen)
python manage.py doc

# Sign binary (macOS only)
python manage.py sign

# Show help
python manage.py help
```

## Troubleshooting

### Boost not found

**Linux:**
```bash
sudo apt-get install libboost-all-dev
```

**macOS:**
```bash
brew install boost
```

**Windows:**
```powershell
# Using vcpkg
.\vcpkg\vcpkg install boost
```

### OMPL not found

**Linux:**
```bash
sudo apt-get install libompl-dev
```

**macOS:**
```bash
brew install ompl
```

**Windows:**
```powershell
# Using vcpkg
.\vcpkg\vcpkg install ompl
```

### CMake version too old

Update CMake to version 3.16 or higher:

**Linux:**
```bash
# Use snap for latest version
sudo snap install cmake --classic
```

**macOS:**
```bash
brew upgrade cmake
```

**Windows:**
Download the latest installer from https://cmake.org/download/

### Build fails with sanitizer errors (Linux/macOS)

The debug build includes address sanitizer by default. If this causes issues:
1. Edit `CMakeLists.txt` and remove the sanitizer flags
2. Or build in release mode: `python manage.py build --release`

### Python not found

Ensure Python 3 is installed and in your PATH:

**Linux:**
```bash
sudo apt-get install python3
```

**macOS:**
```bash
brew install python3
```

**Windows:**
Download from https://www.python.org/downloads/ and ensure "Add to PATH" is checked during installation.

## Getting Help

If you encounter issues not covered here:

1. Check the [README.md](README.md) for general project information
2. Open an issue on the GitHub repository
3. Consult the [CMakeLists.txt](CMakeLists.txt) for build configuration details
