#pragma once
#include <termios.h>
#include <unistd.h>
#include <vector>

class FileSelector {
private:
  std::string folderPath;
  std::vector<std::string> files;
  int selectedIndex;

  void loadFiles();
  char getKeyPress();
  void moveCursorUp();
  void moveCursorDown();
  void displayFiles();

public:
  FileSelector(const std::string &path) : folderPath(path), selectedIndex(0) { loadFiles(); }

  std::string selectFile();
};
