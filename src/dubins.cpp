#include "dubins.h"
#include "utils.h"

Dubins::Dubins(CityGraph::point start, CityGraph::neighbor end)
    : Dubins(start, end, CAR_MAX_SPEED_MS, CAR_MAX_SPEED_MS) {}

Dubins::Dubins(CityGraph::point start, CityGraph::neighbor end, double startSpeed)
    : Dubins(start, end, startSpeed, CAR_MAX_SPEED_MS) {

  // The distance needed to reach the maximum speed
  double distanceToMaxSpeed = (std::pow(CAR_MAX_SPEED_MS, 2) - std::pow(startSpeed, 2)) / (2 * CAR_ACCELERATION);

  double dist = distance();
  if (dist == 0) {
    this->avgSpeed = CAR_MAX_SPEED_MS;
    return;
  }

  if (dist < distanceToMaxSpeed) {
    this->avgSpeed = CAR_MAX_SPEED_MS;
  } else {
    double avg = (startSpeed + CAR_MAX_SPEED_MS) / 2;
    this->avgSpeed = (avg * distanceToMaxSpeed + CAR_MAX_SPEED_MS * (dist - distanceToMaxSpeed)) / dist;
  }
}

Dubins::Dubins(CityGraph::point start, CityGraph::neighbor end, double startSpeed, double endSpeed) {
  this->startPoint = start;
  this->endPoint = end;
  this->startSpeed = startSpeed;
  this->endSpeed = endSpeed;
  this->avgSpeed = (startSpeed + endSpeed) / 2;

  this->space = new ob::DubinsStateSpace(this->endPoint.turningRadius);

  ob::RealVectorBounds bounds(2);
  space->setBounds(bounds);

  this->start = space->allocState();
  this->end = space->allocState();

  this->start->as<ob::DubinsStateSpace::StateType>()->setXY(start.position.x, start.position.y);
  this->start->as<ob::DubinsStateSpace::StateType>()->setYaw(start.angle);

  this->end->as<ob::DubinsStateSpace::StateType>()->setXY(end.point.position.x, end.point.position.y);
  this->end->as<ob::DubinsStateSpace::StateType>()->setYaw(end.point.angle);
}

Dubins::~Dubins() {
  space->freeState(start);
  space->freeState(end);
  delete space;
}

double Dubins::time() { return this->distance() / avgSpeed; }

CityGraph::point Dubins::point(double time) {
  double distance = this->distance();
  double acc = (std::pow(endSpeed, 2) - std::pow(startSpeed, 2)) / (2 * distance);
  auto xFun = [distance, acc, this](double t) { return (0.5 * acc * t * t + this->startSpeed * t) / distance; };

  ob::State *state = space->allocState();
  space->interpolate(start, end, xFun(time), state);

  double x = state->as<ob::DubinsStateSpace::StateType>()->getX();
  double y = state->as<ob::DubinsStateSpace::StateType>()->getY();
  double yaw = state->as<ob::DubinsStateSpace::StateType>()->getYaw();

  space->freeState(state);

  CityGraph::point point;
  point.position = {(float)x, (float)y};
  point.angle = yaw;

  return point;
}

std::vector<CityGraph::point> Dubins::path() {
  std::vector<CityGraph::point> path;
  double time = this->time();
  for (double t = 0; t < time; t += SIM_STEP_TIME) {
    path.push_back(this->point(t));
  }

  return path;
}

DubinsPath::DubinsPath(std::vector<AStar::node> path) : path_(path) {}

std::vector<CityGraph::point> DubinsPath::path() {
  if (pathProcessed_.empty())
    process();

  return pathProcessed_;
}

void DubinsPath::process() {
  pathProcessed_.clear();
  double t = 0;
  double prevTime = 0;

  for (int i = 1; i < (int)path_.size(); i++) {
    AStar::node prevNode = path_[i - 1];
    AStar::node node = path_[i];

    CityGraph::point start = node.arcFrom.first;
    CityGraph::neighbor end = node.arcFrom.second;

    Dubins dubins(start, end, prevNode.speed, node.speed);
    double time = dubins.time();

    if (t >= prevTime + time) {
      continue;
    }

    while (t < prevTime + time) {
      pathProcessed_.push_back(dubins.point(t - prevTime));
      t += SIM_STEP_TIME;
    }

    prevTime += time;
  }
}
