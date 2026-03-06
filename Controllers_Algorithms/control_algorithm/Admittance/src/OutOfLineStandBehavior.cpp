#include "Behavior/OutOfLineStandBehavior.h"

OutOfLineStandBehavior::OutOfLineStandBehavior(const std::string& n)
: force_x(0.0), duration(1.0),
  name_(n), active_(false), elapsed_(0.0)
{
  wrench_.setZero();
}

void OutOfLineStandBehavior::trigger() {
  active_ = true;
  elapsed_ = 0.0;
  wrench_.setZero();
}

void OutOfLineStandBehavior::reset() {
  active_ = false;
  elapsed_ = 0.0;
  wrench_.setZero();
}

void OutOfLineStandBehavior::update(double /*t*/, double dt) {
  if (!active_) { wrench_.setZero(); return; }
  elapsed_ += dt;

  if (elapsed_ <= duration) {
    wrench_.setZero();
    wrench_(0) = force_x;  // lateral direction
  } else {
    wrench_.setZero();
    active_ = false;
  }
}

Vector6d OutOfLineStandBehavior::externalWrench() const {
  return wrench_;
}
