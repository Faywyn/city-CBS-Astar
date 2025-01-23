#include "aStar.h"
#include "config.h"
#include "dubins.h"
#include "utils.h"

#include <spdlog/spdlog.h>
#include <unordered_set>

TimedAStar::TimedAStar(CityGraph::point start, CityGraph::point end, const CityGraph &cityGraph,
                       ConstraintController *conflicts, int carIndex) {
  this->start.point = start;
  this->start.speed = 0;
  this->end.point = end;
  this->end.speed = 0;
  this->graph = cityGraph;
  this->conflicts = conflicts;
  this->carIndex = carIndex;
}

void TimedAStar::process() {
  path.clear();

  std::unordered_map<AStar::node, AStar::node> cameFrom;
  std::unordered_map<AStar::node, double> gScore;
  std::unordered_map<AStar::node, double> fScore;

  auto heuristic = [&](const AStar::node &a) {
    sf::Vector2f diff = end.point.position - a.point.position;
    double distance = std::sqrt(diff.x * diff.x + diff.y * diff.y);
    return distance / CAR_MAX_SPEED_MS;

    CityGraph::neighbor end_;
    end_.point = end.point;
    end_.maxSpeed = CAR_MAX_SPEED_MS;
    end_.turningRadius = CAR_MIN_TURNING_RADIUS;
    Dubins dubins(a.point, end_, CAR_MAX_SPEED_MS, CAR_MAX_SPEED_MS);
    return dubins.time();
  };
  auto compare = [&](const AStar::node &a, const AStar::node &b) { return fScore[a] > fScore[b]; };

  std::priority_queue<AStar::node, std::vector<AStar::node>, decltype(compare)> openSet(compare);
  std::unordered_set<AStar::node> isInOpenSet;

  openSet.push(start);
  gScore[start] = 0;
  fScore[start] = heuristic(start);

  auto neighbors = graph.getNeighbors();

  int nbIterations = 0;
  while (!openSet.empty() && nbIterations++ < 1e5) {
    AStar::node current = openSet.top();
    openSet.pop();
    isInOpenSet.erase(current);

    if (current.point == end.point) {
      AStar::node currentCopy = current;

      while (!(currentCopy == start)) {
        path.push_back(currentCopy);
        currentCopy = cameFrom[currentCopy];
      }
      path.push_back(currentCopy);
      std::reverse(path.begin(), path.end());
      break;
    }

    for (const auto &neighborGraphPoint : neighbors[current.point]) {
      if (current.speed > neighborGraphPoint.maxSpeed)
        continue;

      std::vector<double> newSpeeds;
      newSpeeds.push_back(current.speed);

      double distance = neighborGraphPoint.distance;
      double nSpeedAcc = std::sqrt(std::pow(current.speed, 2) + 2 * CAR_ACCELERATION * distance);
      double nSpeedDec = std::sqrt(std::pow(current.speed, 2) - 2 * CAR_DECELERATION * distance);

      if (nSpeedAcc > neighborGraphPoint.maxSpeed && current.speed < neighborGraphPoint.maxSpeed) {
        newSpeeds.push_back(neighborGraphPoint.maxSpeed);
        newSpeeds.push_back((current.speed + neighborGraphPoint.maxSpeed) / 2);
      } else if (nSpeedAcc < neighborGraphPoint.maxSpeed) {
        newSpeeds.push_back(nSpeedAcc);
        newSpeeds.push_back((current.speed + nSpeedAcc) / 2);
      }

      if (nSpeedDec == nSpeedDec && std::isfinite(nSpeedDec)) {
        if (nSpeedDec < 0 && current.speed > 0) {
          newSpeeds.push_back(0);
          newSpeeds.push_back((current.speed + 0) / 2);
        } else if (nSpeedDec >= 0) {
          newSpeeds.push_back(nSpeedDec);
          newSpeeds.push_back((current.speed + nSpeedDec) / 2);
        }
      }

      AStar::node neighbor;
      neighbor.point = neighborGraphPoint.point;
      neighbor.arcFrom = {current.point, neighborGraphPoint};
      if (distance == 0) {
        neighbor.speed = current.speed;
        if (gScore.find(neighbor) == gScore.end() || gScore[current] < gScore[neighbor]) {
          cameFrom[neighbor] = current;
          gScore[neighbor] = gScore[current];
          fScore[neighbor] = gScore[neighbor] + heuristic(neighbor);

          if (isInOpenSet.find(neighbor) == isInOpenSet.end()) {
            openSet.push(neighbor);
            isInOpenSet.insert(neighbor);
          }
        }
        continue;
      }

      for (const auto &newSpeed : newSpeeds) {
        if (newSpeed > CAR_MAX_SPEED_MS || newSpeed > neighborGraphPoint.maxSpeed || newSpeed < 0)
          continue;

        if (newSpeed == current.speed && newSpeed == 0)
          continue;

        neighbor.speed = newSpeed;

        double duration = 2 * distance / (current.speed + newSpeed);
        double tentativeGScore = gScore[current] + duration;

        double t = gScore[current];

        if (conflicts != nullptr &&
            conflicts->checkConstraints(carIndex, current.speed, newSpeed, t, current.point, neighborGraphPoint))
          continue;

        if (gScore.find(neighbor) == gScore.end() || tentativeGScore < gScore[neighbor]) {
          cameFrom[neighbor] = current;
          gScore[neighbor] = tentativeGScore;
          fScore[neighbor] = gScore[neighbor] + heuristic(neighbor);

          if (isInOpenSet.find(neighbor) == isInOpenSet.end()) {
            openSet.push(neighbor);
            isInOpenSet.insert(neighbor);
          }
        }
      }
    }
  }
}
