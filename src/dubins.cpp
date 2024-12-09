#include "dubins.h"
#include "utils.h"

Dubins::Dubins(CityGraph::point start, CityGraph::neighbor end)
    : Dubins(start, end, CAR_MAX_SPEED_MS, CAR_MAX_SPEED_MS) {}

Dubins::Dubins(CityGraph::point start, CityGraph::neighbor end, float startSpeed, float endSpeed) {
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

float Dubins::distance() { return space->distance(start, end); }
float Dubins::time() { return this->distance() / avgSpeed; }

CityGraph::point Dubins::point(float time) {
  float distance = this->distance();
  float acc = (endSpeed - startSpeed) / this->time();
  auto xFun = [distance, acc, this](float t) { return (0.5f * acc * t * t + this->startSpeed * t) / distance; };

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
    AStar::node prevNode = path_[i - 1];
    AStar::node node = path_[i];

    CityGraph::point start = node.arcFrom.first;
    CityGraph::neighbor end = node.arcFrom.second;

    Dubins dubins(start, end, prevNode.speed, node.speed);
    float time = dubins.time();

    while (t < prevTime + time) {
      pathProcessed_.push_back(dubins.point(t - prevTime));
      t += SIM_STEP_TIME;
    }

    prevTime += time;
  }
}
