#include "Behavior/SeatSlidingBehavior.h"

SeatSlidingBehavior::SeatSlidingBehavior(const std::string& n)
: slide_force_y(30.0), duration(0.8), only_when_lowZ(false), lowZ_threshold(0.0),
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
    wrench_(1) = -slide_force_y; // constant force along y-axis
  } else {
    wrench_.setZero();
    active_ = false;
  }
}

Vector6d SeatSlidingBehavior::externalWrench() const {
  return wrench_;
}