#include "Behavior/OutOfLineStSBackBehavior.h"

OutOfLineStSBackBehavior::OutOfLineStSBackBehavior(const std::string& n)
: force_y(0.0), force_z(0.0), torque_x(0.0), duration(1.0),
  name_(n), active_(false), elapsed_(0.0)
{
  wrench_.setZero();
}

void OutOfLineStSBackBehavior::trigger() {
  active_ = true;
  elapsed_ = 0.0;
  wrench_.setZero();
}

void OutOfLineStSBackBehavior::reset() {
  active_ = false;
  elapsed_ = 0.0;
  wrench_.setZero();
}

void OutOfLineStSBackBehavior::update(double /*t*/, double dt) {
  if (!active_) { wrench_.setZero(); return; }
  elapsed_ += dt;

  if (elapsed_ <= duration) {
    wrench_.setZero();
    wrench_(1) = force_y;   // sagittal direction
    wrench_(2) = force_z;   // vertical direction
    wrench_(3) = torque_x;  // pitch
  } else {
    wrench_.setZero();
    active_ = false;
  }
}

Vector6d OutOfLineStSBackBehavior::externalWrench() const {
  return wrench_;
}
