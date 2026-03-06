#include "Behavior/OutOfLineStSSideBehavior.h"

OutOfLineStSSideBehavior::OutOfLineStSSideBehavior(const std::string& n)
: force_x(0.0), force_y(0.0), force_z(0.0), torque_y(0.0), duration(1.0),
  name_(n), active_(false), elapsed_(0.0)
{
  wrench_.setZero();
}

void OutOfLineStSSideBehavior::trigger() {
  active_ = true;
  elapsed_ = 0.0;
  wrench_.setZero();
}

void OutOfLineStSSideBehavior::reset() {
  active_ = false;
  elapsed_ = 0.0;
  wrench_.setZero();
}

void OutOfLineStSSideBehavior::update(double /*t*/, double dt) {
  if (!active_) { wrench_.setZero(); return; }
  elapsed_ += dt;

  if (elapsed_ <= duration) {
    wrench_.setZero();
    wrench_(0) = force_x;   // lateral direction
    wrench_(1) = force_y;   // sagittal direction
    wrench_(2) = force_z;   // vertical direction
    wrench_(4) = torque_y;  // roll
  } else {
    wrench_.setZero();
    active_ = false;
  }
}

Vector6d OutOfLineStSSideBehavior::externalWrench() const {
  return wrench_;
}
