/**
 * @file fileSelector.h
 * @brief File selector
 *
 * This file contains the FileSelector class. It is used to select a file from a folder.
 */
#pragma once
#include <iostream>
#include <termios.h>
#include <unistd.h>
#include <vector>

/**
 * @class FileSelector
 * @brief A file selector
 *
 * This class represents a file selector. It allows the user to select a file from a folder.
 */
class FileSelector {
private:
  std::string folderPath;         /**< \brief The folder path */
  std::vector<std::string> files; /**< \brief The files in the folder */
  int selectedIndex;              /**< \brief The selected index */

  void loadFiles();      /**< \brief Load the files in the folder */
  char getKeyPress();    /**< \brief Get a key press */
  void moveCursorUp();   /**< \brief Move the cursor up */
  void moveCursorDown(); /**< \brief Move the cursor down */
  void displayFiles();   /**< \brief Display the files */

public:
  FileSelector(const std::string &path) : folderPath(path), selectedIndex(0) { loadFiles(); }
  ~FileSelector() { std::cout << "\033[?25h"; }

  std::string selectFile();
};
