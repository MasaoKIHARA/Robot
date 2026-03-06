#include "Behavior/KneeBucklingBehavior.h"

KneeBucklingBehavior::KneeBucklingBehavior(const std::string& n)
: force_y(-80.0), force_z(-80.0), duration(0.3),
  name_(n), active_(false), elapsed_(0.0)
{
  wrench_.setZero();
}

void KneeBucklingBehavior::trigger() {
  active_ = true;
  elapsed_ = 0.0;
  wrench_.setZero();
}

void KneeBucklingBehavior::reset() {
  active_ = false;
  elapsed_ = 0.0;
  wrench_.setZero();
}

void KneeBucklingBehavior::update(double /*t*/, double dt) {
  if (!active_) { wrench_.setZero(); return; }
  elapsed_ += dt;

  if (elapsed_ <= duration) {
    wrench_.setZero();
    wrench_(1) = force_y; // sagittal direction
    wrench_(2) = force_z; // vertical direction
  }
  else {
    wrench_.setZero();
    active_ = false;
  }
}

Vector6d KneeBucklingBehavior::externalWrench() const {
  return wrench_;
}
