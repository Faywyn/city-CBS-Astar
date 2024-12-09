#include <algorithm>
#include <iostream>
#include <ompl/base/State.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <random>
#include <spdlog/spdlog.h>
#include <vector>

#include "aStar.h"
#include "config.h"
#include "renderer.h"
#include "utils.h"

namespace ob = ompl::base;

void Renderer::startRender(const CityMap &cityMap, const CityGraph &cityGraph, Manager &manager) {
  window.create(sf::VideoMode(SCREEN_WIDTH, SCREEN_HEIGHT), "City Map");
  window.setFramerateLimit(60);

  // Set the view to the center of the city map, allowing some basic camera movement
  // Arrow to move the camera, + and - to zoom in and out
  float height = cityMap.getHeight();
  float width = cityMap.getWidth();
  sf::View view(sf::FloatRect(0, 0, width, height));
  // Reset view function
  auto resetView = [&]() {
    float screenRatio = window.getSize().x / (float)window.getSize().y;
    float cityRatio = width / height;
    view.setCenter(width / 2, height / 2);
    if (screenRatio > cityRatio) {
      view.setSize(height * screenRatio, height);
    } else {
      view.setSize(width, width / screenRatio);
    }
    window.setView(view);
  };

  resetView();
  time.restart();

  sf::Clock clockCars;

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
    if (clockCars.getElapsedTime().asMilliseconds() > SIM_STEP_TIME) {
      manager.moveCars();
      clockCars.restart();
    }
    if (debug) {
      renderCityGraph(cityGraph, view);
    }
    // Remove outside the border (draw blank)
    sf::RectangleShape rectangle(sf::Vector2f(width, height));
    rectangle.setFillColor(sf::Color(247, 246, 242));

    std::vector<sf::Vector2f> border = {{-width, -height}, {0, -height}, {width, -height}, {width, 0},
                                        {width, height},   {0, height},  {-width, height}, {-width, 0}};
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
  for (const auto &road : cityMap.getRoads()) {
    for (const auto &segment : road.segments) {
      sf::Vector2f basedP1(segment.p1.x, segment.p1.y);
      sf::Vector2f basedP2(segment.p2.x, segment.p2.y);

      float angle = segment.angle;

      sf::Vector2f widthVec(sin(angle), -cos(angle));
      widthVec *= road.width / 2;

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

      convex.setFillColor(sf::Color(155, 162, 163));

      window.draw(convex);

      // Draw a circle at the start end end of the road (for filling the gap)
      float radius = road.width / 2;
      sf::CircleShape circle(radius);
      circle.setFillColor(sf::Color(155, 162, 163));
      circle.setPosition(basedP1.x - radius, basedP1.y - radius);
      window.draw(circle);
      circle.setPosition(basedP2.x - radius, basedP2.y - radius);
      window.draw(circle);
    }
  }

  // Draw intersections
  if (debug) {
    for (const auto &intersection : cityMap.getIntersections()) {
      float radius = intersection.radius;
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

      float radius = turningRadius(neighbor.maxSpeed);
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
      float step = CELL_SIZE / 2.0f;
      float distance = space.distance(start, end);
      int numSteps = distance / step;
      sf::Vector2f lastPosition;

      for (int k = 0; k < numSteps; k++) {
        if (k == 0) {
          lastPosition = {point.position.x, point.position.y};
          continue;
        }

        ob::State *state = space.allocState();
        space.interpolate(start, end, (float)k / (float)numSteps, state);

        float x = state->as<ob::DubinsStateSpace::StateType>()->getX();
        float y = state->as<ob::DubinsStateSpace::StateType>()->getY();

        float distance = std::sqrt(std::pow(x - lastPosition.x, 2) + std::pow(y - lastPosition.y, 2));
        float angle = atan2(y - lastPosition.y, x - lastPosition.x) * 180 / M_PI;

        // Draw an arrow between the points
        drawArrow(window, lastPosition, angle, distance / 2, distance / 4, sf::Color(0, 0, 255, 50), false);

        lastPosition = {x, y};
      }

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

    // Draw an arrow at each points
    drawArrow(window, point.position, point.angle * 180 / M_PI, 1, 0.3, sf::Color(255, 0, 0, 50), true);
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
  text.setString(std::to_string((int)time.getElapsedTime().asSeconds()) + " s");
  text.setOutlineColor(sf::Color::Black);
  text.setOutlineThickness(1.0f);
  text.scale(viewSize.x * 0.001f, viewSize.x * 0.001f);
  text.setOrigin(text.getLocalBounds().width, 0);
  window.draw(text);
}
