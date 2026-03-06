#include "Behavior/SeatSlidingBehavior.h"

SeatSlidingBehavior::SeatSlidingBehavior(const std::string& n)
: force_y(0.0), force_z(0.0), torque_x(0.0), duration(2.0),
  name_(n), active_(false), elapsed_(0.0)
{
  wrench_.setZero();
}

void SeatSlidingBehavior::trigger() {
  active_ = true;
  elapsed_ = 0.0;
  wrench_.setZero();
}

void SeatSlidingBehavior::reset() {
  active_ = false;
  elapsed_ = 0.0;
  wrench_.setZero();
}

void SeatSlidingBehavior::update(double /*t*/, double dt) {
  if (!active_) { wrench_.setZero(); return; }
  elapsed_ += dt;

  if (elapsed_ <= duration) {
    wrench_.setZero();
    wrench_(1) = force_y;   // y-axis
    wrench_(2) = force_z;   // z-axis
    wrench_(3) = torque_x;  // pitch
  } else {
    wrench_.setZero();
    active_ = false;
  }
}

Vector6d SeatSlidingBehavior::externalWrench() const {
  return wrench_;
}