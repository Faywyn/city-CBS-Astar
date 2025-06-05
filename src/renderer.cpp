/**
 * @file renderer.cpp
 * @brief Implementation of the Renderer class
 *
 * This file contains the implementation of the Renderer class.
 */
#include "renderer.h"
#include "config.h"
#include "utils.h"
#include <ompl/base/State.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <spdlog/spdlog.h>
#include <vector>

namespace ob = ompl::base;

void Renderer::startRender(const CityMap &cityMap, const CityGraph &cityGraph, Manager &manager) {
  manager.planPaths();

  window.create(sf::VideoMode({SCREEN_WIDTH, SCREEN_HEIGHT}), "City Map");

  // Set the view to the center of the city map, allowing some basic camera movement
  // Arrow to move the camera, + and - to zoom in and out
  double height = cityMap.getHeight();
  double width = cityMap.getWidth();
  sf::View view(sf::FloatRect({0, 0}, {(float)width, (float)height}));
  // Reset view function
  auto resetView = [&]() {
    double screenRatio = window.getSize().x / (double)window.getSize().y;
    double cityRatio = width / height;
    view.setCenter({(float)width / 2, (float)height / 2});
    if (screenRatio > cityRatio) {
      view.setSize({(float)(height * screenRatio), (float)height});
    } else {
      view.setSize({(float)width, (float)(width / screenRatio)});
    }
    window.setView(view);
  };

  resetView();
  renderCityMap(cityMap);
  window.display();
  time = 0;

  sf::Clock clockCars;
  bool speedUp = false;
  bool pause = true;

  while (true) {
    while (const std::optional event = window.pollEvent()) {
      if (event->is<sf::Event::Closed>()) {
        window.close();
        return;
      }

      if (event->is<sf::Event::KeyPressed>() || event->is<sf::Event::MouseButtonPressed>()) {
        manager.userInput(event.value(), window);
      }

      if (const auto *resized = event->getIf<sf::Event::Resized>()) {
        resetView();
      }

      if (!event->is<sf::Event::KeyPressed>())
        continue;

      if (event->getIf<sf::Event::KeyPressed>()->code == sf::Keyboard::Key::Escape) {
        window.close();
      } else if (event->getIf<sf::Event::KeyPressed>()->code == sf::Keyboard::Key::Up) {
        view.move({0, -(float)(height * MOVE_SPEED)});
      } else if (event->getIf<sf::Event::KeyPressed>()->code == sf::Keyboard::Key::Down) {
        view.move({0, +(float)(height * MOVE_SPEED)});
      } else if (event->getIf<sf::Event::KeyPressed>()->code == sf::Keyboard::Key::Left) {
        view.move({-(float)(width * MOVE_SPEED), 0});
      } else if (event->getIf<sf::Event::KeyPressed>()->code == sf::Keyboard::Key::Right) {
        view.move({+(float)(width * MOVE_SPEED), 0});
      } else if (event->getIf<sf::Event::KeyPressed>()->code == sf::Keyboard::Key::Equal) {
        view.zoom(1.0f - ZOOM_SPEED);
      } else if (event->getIf<sf::Event::KeyPressed>()->code == sf::Keyboard::Key::Subtract) {
        view.zoom(1.0f + ZOOM_SPEED);
      } else if (event->getIf<sf::Event::KeyPressed>()->code == sf::Keyboard::Key::R) {
        resetView();
        spdlog::debug("View reset");
      } else if (event->getIf<sf::Event::KeyPressed>()->code == sf::Keyboard::Key::D) {
        debug = !debug;
        spdlog::debug("Debug mode: {}", debug);
      } else if (event->getIf<sf::Event::KeyPressed>()->code == sf::Keyboard::Key::S) {
        speedUp = !speedUp;
      } else if (event->getIf<sf::Event::KeyPressed>()->code == sf::Keyboard::Key::P) {
        pause = !pause;
      }
    }

    window.setView(view);
    window.clear(sf::Color(247, 246, 242));
    renderCityMap(cityMap);
    renderManager(manager);
    if (!pause) {
      if (clockCars.getElapsedTime().asSeconds() > SIM_STEP_TIME ||
          (speedUp && clockCars.getElapsedTime().asSeconds() > SIM_STEP_TIME / 5)) {
        time += SIM_STEP_TIME;
        manager.updateAgents();
        clockCars.restart();
      }
    }
    if (debug) {
      renderCityGraph(cityGraph, view);
    }
    // Remove outside the border (draw blank)
    sf::RectangleShape rectangle(sf::Vector2f(width, height));
    rectangle.setFillColor(sf::Color(247, 246, 242));

    float w = width;
    float h = height;

    std::vector<sf::Vector2f> border = {{-w, -h}, {0, -h}, {w, -h}, {w, 0}, {w, h}, {0, h}, {-w, h}, {-w, 0}};
    for (auto b : border) {
      rectangle.setPosition(b);
      window.draw(rectangle);
    }

    renderTime();
    window.display();
  }
}

void Renderer::renderCityMap(const CityMap &cityMap) {
  // Draw buildings
  std::vector<sf::Color> randomBuildingColors = {
      sf::Color(233, 234, 232), sf::Color(238, 231, 210), sf::Color(230, 229, 226), sf::Color(236, 234, 230),
      sf::Color(230, 223, 216), sf::Color(230, 234, 236), sf::Color(210, 215, 222)};

  std::vector<sf::Color> greenAreaColor = {sf::Color(184, 230, 144), sf::Color(213, 240, 193)};

  sf::Color waterColor(139, 214, 245);

  auto greenAreas = cityMap.getGreenAreas();
  for (int i = 0; i < (int)greenAreas.size(); i++) {
    const auto &greenArea = greenAreas[i];
    auto points = greenArea.points;
    sf::ConvexShape convex;
    convex.setPointCount(points.size());
    for (size_t i = 0; i < points.size(); i++) {
      convex.setPoint(i, sf::Vector2f(points[i].x, points[i].y));
    }
    convex.setFillColor(greenAreaColor[greenArea.type]);

    window.draw(convex);
  }

  auto waterAreas = cityMap.getWaterAreas();
  for (int i = 0; i < (int)waterAreas.size(); i++) {
    const auto &waterArea = waterAreas[i];
    auto points = waterArea.points;
    sf::ConvexShape convex;
    convex.setPointCount(points.size());
    for (size_t i = 0; i < points.size(); i++) {
      convex.setPoint(i, sf::Vector2f(points[i].x, points[i].y));
    }
    convex.setFillColor(waterColor);

    window.draw(convex);
  }

  auto buildings = cityMap.getBuildings();
  for (int i = 0; i < (int)buildings.size(); i++) {
    const auto &building = buildings[i];
    auto points = building.points;
    sf::ConvexShape convex;
    convex.setPointCount(points.size());
    for (size_t i = 0; i < points.size(); i++) {
      convex.setPoint(i, sf::Vector2f(points[i].x, points[i].y));
    }
    convex.setFillColor(randomBuildingColors[i % randomBuildingColors.size()]);

    window.draw(convex);
  }

  // Draw roads
  sf::Color roadColor(194, 201, 202);
  for (const auto &road : cityMap.getRoads()) {
    for (const auto &segment : road.segments) {
      sf::Vector2f basedP1(segment.p1.x, segment.p1.y);
      sf::Vector2f basedP2(segment.p2.x, segment.p2.y);

      sf::Angle angle = segment.angle;

      sf::Vector2f widthVec({sin(angle.asRadians()), -cos(angle.asRadians())});
      widthVec *= (float)road.width / 2;

      sf::Vector2f p1 = basedP1 + widthVec;
      sf::Vector2f p2 = basedP1 - widthVec;
      sf::Vector2f p3 = basedP2 - widthVec;
      sf::Vector2f p4 = basedP2 + widthVec;

      sf::ConvexShape convex;
      convex.setPointCount(4);
      convex.setPoint(0, p1);
      convex.setPoint(1, p2);
      convex.setPoint(2, p3);
      convex.setPoint(3, p4);

      convex.setFillColor(roadColor);

      window.draw(convex);

      // Draw a circle at the start end end of the road (for filling the gap)
      double radius = road.width / 2;
      sf::CircleShape circle(radius);
      circle.setFillColor(roadColor);
      circle.setPosition({(float)(basedP1.x - radius), (float)(basedP1.y - radius)});
      window.draw(circle);
      circle.setPosition({(float)(basedP2.x - radius), (float)(basedP2.y - radius)});
      window.draw(circle);
    }
  }

  // Draw intersections
  if (debug) {
    for (const auto &intersection : cityMap.getIntersections()) {
      double radius = intersection.radius;
      sf::CircleShape circle(radius);
      circle.setFillColor(sf::Color(0, 255, 0, 50));
      circle.setPosition({(float)(intersection.center.x - radius), (float)(intersection.center.y - radius)});
      window.draw(circle);
    }
  }
}

void Renderer::renderCityGraph(const CityGraph &cityGraph, const sf::View &view) {
  std::unordered_set<CityGraph::point> graphPoints = cityGraph.getGraphPoints();
  std::unordered_map<CityGraph::point, std::vector<CityGraph::neighbor>> neighbors = cityGraph.getNeighbors();

  // Draw a line between each point and its neighbors
  for (const auto &point : graphPoints) {
    for (const auto &neighbor : neighbors[point]) {
      if (!neighbor.isRightWay)
        continue;

      double radius = turningRadius(neighbor.maxSpeed);
      auto space = ob::DubinsStateSpace(radius, true);
      ob::RealVectorBounds bounds(2);
      space.setBounds(bounds);

      // Draw only if one of the points is inside the view
      sf::Vector2f viewCenter = view.getCenter();
      sf::Vector2f viewSize = view.getSize();
      sf::Vector2f viewMin = viewCenter - viewSize / 2.0f;
      sf::Vector2f viewMax = viewCenter + viewSize / 2.0f;

      if (point.position.x < viewMin.x && neighbor.point.position.x < viewMin.x) {
        continue;
      }
      if (point.position.x > viewMax.x && neighbor.point.position.x > viewMax.x) {
        continue;
      }

      ob::State *start = space.allocState();
      ob::State *end = space.allocState();

      start->as<ob::DubinsStateSpace::StateType>()->setXY(point.position.x, point.position.y);
      start->as<ob::DubinsStateSpace::StateType>()->setYaw(point.angle.asRadians());

      end->as<ob::DubinsStateSpace::StateType>()->setXY(neighbor.point.position.x, neighbor.point.position.y);
      end->as<ob::DubinsStateSpace::StateType>()->setYaw(neighbor.point.angle.asRadians());

      // Draw the Dubins curve
      double step = CELL_SIZE / 2.0f;
      double distance = space.distance(start, end);
      int numSteps = distance / step;
      sf::Vector2f lastPosition;
      sf::Color randomColor = sf::Color(rand() % 255, rand() % 255, rand() % 255, 60);

      for (int k = 0; k < numSteps; k++) {
        if (k == 0) {
          lastPosition = {point.position.x, point.position.y};
          continue;
        }

        ob::State *state = space.allocState();
        space.interpolate(start, end, (double)k / (double)numSteps, state);

        double x = state->as<ob::DubinsStateSpace::StateType>()->getX();
        double y = state->as<ob::DubinsStateSpace::StateType>()->getY();

        double distance = std::sqrt(std::pow(x - lastPosition.x, 2) + std::pow(y - lastPosition.y, 2));
        sf::Angle angle = sf::radians(atan2(y - lastPosition.y, x - lastPosition.x));

        // Draw an arrow between the points
        drawArrow(window, lastPosition, angle, distance * 0.9, distance * 0.9 / 2, randomColor, false);

        lastPosition = {(float)x, (float)y};
      }

      continue;
      // Write the speed of the point
      sf::Font font = loadFont();
      sf::Text text(font);
      text.setString(std::to_string((int)(neighbor.maxSpeed * 3.6f)) + " km/h");
      text.setCharacterSize(24);
      text.setFillColor(sf::Color::Black);
      text.setOutlineColor(sf::Color::White);
      text.setOutlineThickness(1.0f);
      text.setPosition(point.position * 0.2f + neighbor.point.position * 0.8f);
      text.setScale({0.02f, 0.02f});
      text.setOrigin({text.getLocalBounds().size.x / 2.0f, text.getLocalBounds().size.y / 2.0f});
      window.draw(text);
    }

    // Draw a dot at each points
    double size = 0.3;
    sf::CircleShape circle(size);
    circle.setFillColor(sf::Color(255, 0, 0, 70));
    circle.setPosition({(float)(point.position.x - size), (float)(point.position.y - size)});
    window.draw(circle);
  }
}

void Renderer::renderManager(Manager &manager) { manager.renderAgents(window); }

void Renderer::renderTime() {
  // At the top right corner of the view (keep the same size even if the view is resized)
  sf::Font font = loadFont();
  sf::Text text(font);
  sf::Vector2f viewSize = window.getView().getSize();
  text.setCharacterSize(24);
  text.setFillColor(sf::Color::White);
  text.setPosition(window.getView().getCenter() + sf::Vector2f(viewSize.x / 2, -viewSize.y / 2) +
                   sf::Vector2f(-viewSize.x * 0.01f, viewSize.y * 0.01f));
  text.setString(std::to_string((int)time) + " s");
  text.setOutlineColor(sf::Color::Black);
  text.setOutlineThickness(1.0f);
  text.scale({viewSize.x * 0.001f, viewSize.x * 0.001f});
  text.setOrigin({text.getLocalBounds().size.x, 0});
  window.draw(text);
}
