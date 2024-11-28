#include "test.h"
#include "spdlog/spdlog.h"
#include "tinyxml2.h"
#include <SFML/Graphics.hpp>

void Test::runTests() {
  testSpdlog();
  testTinyXML2();
  testSFML();
}

void Test::testSpdlog() {
  try {
    spdlog::debug("Testing spdlog...");
    spdlog::info("spdlog is working as expected.");
  } catch (const std::exception &e) {
    throw std::runtime_error("spdlog is not working as expected.");
  }
}

void Test::testTinyXML2() {
  try {
    spdlog::debug("Testing TinyXML2...");
    tinyxml2::XMLDocument xmlDoc;
    xmlDoc.Parse("<root></root>");
    if (xmlDoc.Error()) {
      spdlog::error("TinyXML2 is not working as expected.");
      throw std::runtime_error("TinyXML2 is not working as expected.");
    }
    spdlog::debug("TinyXML2 is working as expected.");
  } catch (const std::exception &e) {
    spdlog::error("TinyXML2 is not working as expected.");
    throw std::runtime_error("TinyXML2 is not working as expected.");
  }
}

void Test::testSFML() {
  try {
    spdlog::debug("Testing SFML...");
    sf::RenderWindow window(sf::VideoMode(100, 100), "Test");
    if (!window.isOpen()) {
      spdlog::error("SFML is not working as expected.");
      throw std::runtime_error("SFML is not working as expected.");
    }
    window.close();
    spdlog::debug("SFML is working as expected.");
  } catch (const std::exception &e) {
    spdlog::error("SFML is not working as expected.");
    throw std::runtime_error("SFML is not working as expected.");
  }
}
