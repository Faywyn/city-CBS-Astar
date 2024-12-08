#include "dubins.h"
#include "utils.h"

Dubins::Dubins(AStar::node start, AStar::node end) {
  this->startNode = start;
  this->endNode = end;

  if (end.speed < 0 || start.speed < 0) {
    this->avgSpeed = CAR_MAX_SPEED_MS;
    this->radius = CAR_MIN_TURNING_RADIUS;
  } else {
    this->avgSpeed = (start.speed + end.speed) / 2;
    this->radius = turningRadius(this->avgSpeed);
  }

  this->space = new ob::DubinsStateSpace(this->radius);

  ob::RealVectorBounds bounds(2);
  space->setBounds(bounds);

  this->start = space->allocState();
  this->end = space->allocState();

  this->start->as<ob::DubinsStateSpace::StateType>()->setXY(start.position.x, start.position.y);
  this->start->as<ob::DubinsStateSpace::StateType>()->setYaw(start.angle);

  this->end->as<ob::DubinsStateSpace::StateType>()->setXY(end.position.x, end.position.y);
  this->end->as<ob::DubinsStateSpace::StateType>()->setYaw(end.angle);
}

Dubins::~Dubins() {
  space->freeState(start);
  space->freeState(end);
  delete space;
}

float Dubins::distance() { return space->distance(start, end); }
float Dubins::time() { return this->distance() / avgSpeed; }

CityGraph::point Dubins::point(float time) {
  float distance = this->distance();
  float acc = (endNode.speed - startNode.speed) / this->time();
  float speedStart = startNode.speed;
  auto xFun = [distance, acc, speedStart](float t) { return (0.5f * acc * t * t + speedStart * t) / distance; };

  ob::State *state = space->allocState();
  space->interpolate(start, end, xFun(time), state);

  float x = state->as<ob::DubinsStateSpace::StateType>()->getX();
  float y = state->as<ob::DubinsStateSpace::StateType>()->getY();
  float yaw = state->as<ob::DubinsStateSpace::StateType>()->getYaw();

  space->freeState(state);

  CityGraph::point point;
  point.position = {x, y};
  point.angle = yaw;

  return point;
}

std::vector<CityGraph::point> Dubins::path() {
  std::vector<CityGraph::point> path;
  float time = this->time();
  for (float t = 0; t < time; t += SIM_STEP_TIME) {
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
  float t = 0;
  float prevTime = 0;

  for (int i = 1; i < (int)path_.size(); i++) {
    AStar::node prev = path_[i - 1];
    AStar::node current = path_[i];

    Dubins dubins(prev, current);

    float time = dubins.time();

    while (t < prevTime + time) {
      pathProcessed_.push_back(dubins.point(t - prevTime));
      t += SIM_STEP_TIME;
    }

    prevTime += time;
  }
}
