#include <spdlog/spdlog.h>

#include "utils.h"

bool fontLoaded = false;
sf::Font font;

sf::Font loadFont() {
  if (!fontLoaded) {
    if (!font.loadFromFile("assets/fonts/arial.ttf")) {
      spdlog::error("Failed to load font");
    }
    fontLoaded = true;
  }
  return font;
}
