#include <algorithm>
#include <iostream>
#include <random>
#include <vector>

#include <ompl/base/State.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <spdlog/spdlog.h>

#include "aStar.h"
#include "config.h"
#include "renderer.h"
#include "utils.h"

namespace ob = ompl::base;

void Renderer::startRender(const CityMap &cityMap, const CityGraph &cityGraph, Manager &manager) {
  window.create(sf::VideoMode(SCREEN_WIDTH, SCREEN_HEIGHT), "City Map");

  // Set the view to the center of the city map, allowing some basic camera movement
  // Arrow to move the camera, + and - to zoom in and out
  double height = cityMap.getHeight();
  double width = cityMap.getWidth();
  sf::View view(sf::FloatRect(0, 0, width, height));
  // Reset view function
  auto resetView = [&]() {
    double screenRatio = window.getSize().x / (double)window.getSize().y;
    double cityRatio = width / height;
    view.setCenter(width / 2, height / 2);
    if (screenRatio > cityRatio) {
      view.setSize(height * screenRatio, height);
    } else {
      view.setSize(width, width / screenRatio);
    }
    window.setView(view);
  };

  resetView();
  time = 0;

  sf::Clock clockCars;
  bool speedUp = false;
  bool pause = true;

  while (true) {
    sf::Event event;
    while (window.pollEvent(event)) {
      if (event.type == sf::Event::Closed) {
        window.close();
        return;
      }

      if (event.type == sf::Event::MouseButtonPressed) {
        if (event.mouseButton.button == sf::Mouse::Left) {
          sf::Vector2f mousePos = window.mapPixelToCoords(sf::Mouse::getPosition(window));
          manager.toggleCarDebug(mousePos);
        }
      }

      if (event.type == sf::Event::KeyPressed) {
        if (event.key.code == sf::Keyboard::Escape) {
          window.close();
          return;
        }
        if (event.key.code == sf::Keyboard::Up) {
          view.move(0, -height * MOVE_SPEED);
        }
        if (event.key.code == sf::Keyboard::Down) {
          view.move(0, height * MOVE_SPEED);
        }
        if (event.key.code == sf::Keyboard::Left) {
          view.move(-width * MOVE_SPEED, 0);
        }
        if (event.key.code == sf::Keyboard::Right) {
          view.move(width * MOVE_SPEED, 0);
        }
        if (event.key.code == sf::Keyboard::Equal) {
          view.zoom(1.0f - ZOOM_SPEED);
        }
        if (event.key.code == sf::Keyboard::Dash) {
          view.zoom(1.0f + ZOOM_SPEED);
        }
        if (event.key.code == sf::Keyboard::R) {
          resetView();
          spdlog::debug("View reset");
        }
        if (event.key.code == sf::Keyboard::D) {
          debug = !debug;
          spdlog::debug("Debug mode: {}", debug);
        }
        if (event.key.code == sf::Keyboard::S) {
          speedUp = !speedUp;
        }
        if (event.key.code == sf::Keyboard::P) {
          pause = !pause;
        }
      }

      // If resizing the window, reset the view
      if (event.type == sf::Event::Resized) {
        resetView();
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
        manager.moveCars();
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

      double angle = segment.angle;

      sf::Vector2f widthVec(sin(angle), -cos(angle));
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
      circle.setPosition(basedP1.x - radius, basedP1.y - radius);
      window.draw(circle);
      circle.setPosition(basedP2.x - radius, basedP2.y - radius);
      window.draw(circle);
    }
  }

  // Draw intersections
  if (debug) {
    for (const auto &intersection : cityMap.getIntersections()) {
      double radius = intersection.radius;
      sf::CircleShape circle(radius);
      circle.setFillColor(sf::Color(0, 255, 0, 50));
      circle.setPosition(intersection.center.x - radius, intersection.center.y - radius);
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
      auto space = ob::DubinsStateSpace(radius);
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
      start->as<ob::DubinsStateSpace::StateType>()->setYaw(point.angle);

      end->as<ob::DubinsStateSpace::StateType>()->setXY(neighbor.point.position.x, neighbor.point.position.y);
      end->as<ob::DubinsStateSpace::StateType>()->setYaw(neighbor.point.angle);

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
        double angle = atan2(y - lastPosition.y, x - lastPosition.x) * 180 / M_PI;

        // Draw an arrow between the points
        drawArrow(window, lastPosition, angle, distance * 0.9, distance * 0.9 / 2, randomColor, false);

        lastPosition = {(float)x, (float)y};
      }

      continue;
      // Write the speed of the point
      sf::Text text;
      sf::Font font;
      font.loadFromFile("assets/fonts/arial.ttf");
      text.setFont(font);
      text.setString(std::to_string((int)(neighbor.maxSpeed * 3.6f)) + " km/h");
      text.setCharacterSize(24);
      text.setFillColor(sf::Color::Black);
      text.setOutlineColor(sf::Color::White);
      text.setOutlineThickness(1.0f);
      text.setPosition(point.position * 0.2f + neighbor.point.position * 0.8f);
      text.setScale(0.02f, 0.02f);
      text.setOrigin(text.getLocalBounds().width / 2.0f, text.getLocalBounds().height / 2.0f);
      window.draw(text);
    }

    // Draw a dot at each points
    double size = 0.3;
    sf::CircleShape circle(size);
    circle.setFillColor(sf::Color(255, 0, 0, 70));
    circle.setPosition(point.position.x - size, point.position.y - size);
    window.draw(circle);
  }
}

void Renderer::renderManager(Manager &manager) { manager.renderCars(window); }

void Renderer::renderTime() {
  // At the top right corner of the view (keep the same size even if the view is resized)
  sf::Text text;
  sf::Font font = loadFont();
  sf::Vector2f viewSize = window.getView().getSize();
  text.setFont(font);
  text.setCharacterSize(24);
  text.setFillColor(sf::Color::White);
  text.setPosition(window.getView().getCenter() + sf::Vector2f(viewSize.x / 2, -viewSize.y / 2) +
                   sf::Vector2f(-viewSize.x * 0.01f, viewSize.y * 0.01f));
  text.setString(std::to_string((int)time) + " s");
  text.setOutlineColor(sf::Color::Black);
  text.setOutlineThickness(1.0f);
  text.scale(viewSize.x * 0.001f, viewSize.x * 0.001f);
  text.setOrigin(text.getLocalBounds().width, 0);
  window.draw(text);
}
