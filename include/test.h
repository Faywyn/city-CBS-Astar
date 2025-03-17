/**
 * @file test.h
 * @brief A header file for the Test class
 */
#pragma once

/**
 * @class Test
 * @brief A class for testing the project
 *
 * This class is used to test the project.
 */
class Test {
public:
  /**
   * @brief Run the tests
   */
  void runTests();

private:
  void testSpdlog();
  void testTinyXML2();
  void testSFML();
};
