/**
 * @file fileSelector.cpp
 * @brief File selector implementation
 *
 * This file contains the implementation of the FileSelector class. It is used to select a file from a folder.
 */
#include "fileSelector.h"
#include "config.h"
#include <filesystem>
#include <spdlog/spdlog.h>

namespace fs = std::filesystem;

void FileSelector::loadFiles() {
  files.clear();
  
  // Check if directory exists
  if (!fs::exists(folderPath)) {
    spdlog::error("Directory does not exist: {}", folderPath);
    return;
  }
  
  if (!fs::is_directory(folderPath)) {
    spdlog::error("Path is not a directory: {}", folderPath);
    return;
  }
  
  // Load all .osm files from directory
  try {
    for (const auto &entry : fs::directory_iterator(folderPath)) {
      if (entry.is_regular_file() && entry.path().extension() == ".osm") {
        files.push_back(entry.path().filename().string());
      }
    }
    std::sort(files.begin(), files.end());
    
    if (files.empty()) {
      spdlog::warn("No .osm files found in directory: {}", folderPath);
    }
  } catch (const fs::filesystem_error &e) {
    spdlog::error("Error reading directory {}: {}", folderPath, e.what());
  }
}

char FileSelector::getKeyPress() {
  struct termios oldt, newt;
  char ch;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  ch = getchar();
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  return ch;
}

void FileSelector::moveCursorUp() {
  if (selectedIndex > 0) {
    std::cout << "\033[2K\r  " << files[selectedIndex] << std::flush;
    selectedIndex--;
    std::cout << "\033[A\033[2K\r> " << files[selectedIndex] << std::flush;
  }
}

void FileSelector::moveCursorDown() {
  if (selectedIndex < files.size() - 1) {
    std::cout << "\033[2K\r  " << files[selectedIndex] << std::flush;
    selectedIndex++;
    std::cout << "\033[B\033[2K\r> " << files[selectedIndex] << std::flush;
  }
}

void FileSelector::displayFiles() {
  std::cout << "Use UP/DOWN arrow keys to navigate, ENTER to select:\n";
  for (size_t i = 0; i < files.size(); i++) {
    if (i == selectedIndex) {
      std::cout << "> " << files[i] << "\n";
    } else {
      std::cout << "  " << files[i] << "\n";
    }
  }
  std::cout << "\033[" << files.size() << "A";
}

std::string FileSelector::selectFile() {
  std::cout << "\033[?25l";
  if (files.empty()) {
    spdlog::error("No .osm files found in the folder: {}", folderPath);
    return "";
  }

  displayFiles();

  while (true) {
    char key = getKeyPress();
    if (key == 27) {
      if (getKeyPress() == '[') {
        switch (getKeyPress()) {
        case 'A':
          moveCursorUp();
          break;
        case 'B':
          moveCursorDown();
          break;
        }
      }
    } else if (key == '\n') {
      std::cout << "\033[" << selectedIndex + 1 << "A\033[2K\r" << std::flush;
      std::cout << "\033[?25h";
      spdlog::info("Selected file: {}", files[selectedIndex]);
      return files[selectedIndex];
    }
  }
}
