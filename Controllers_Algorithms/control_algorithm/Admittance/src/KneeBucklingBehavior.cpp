#include "Behavior/KneeBucklingBehavior.h"
#include <algorithm>

KneeBucklingBehavior::KneeBucklingBehavior(const std::string& n)
: impulse_force_y(-80.0),impulse_force_z(-80.0), impulse_duration(0.03),
  b_fall_time(0.05),name_(n), active_(false), elapsed_(0.0), b_scale_(1.0)
{
  wrench_.setZero();
}

void KneeBucklingBehavior::trigger() {
  active_ = true;
  elapsed_ = 0.0;
  b_scale_ = 1.0;
  wrench_.setZero();
}

void KneeBucklingBehavior::reset() {
  active_ = false;
  elapsed_ = 0.0;
  b_scale_ = 1.0;
  wrench_.setZero();
}

void KneeBucklingBehavior::update(double /*t*/, double dt) {
  if (!active_) { wrench_.setZero(); return; }
  elapsed_ += dt;

  if (elapsed_ <= impulse_duration) {
    wrench_.setZero();
    wrench_(1) = impulse_force_y; // sagittal direction
    wrench_(2) = impulse_force_z; // vertical direction
  }
  else if (elapsed_ <= b_fall_time + impulse_duration) {
    wrench_.setZero();
    wrench_(1) = impulse_force_y * (b_fall_time + impulse_duration - elapsed_)/b_fall_time;
    wrench_(2) = impulse_force_z * (b_fall_time + impulse_duration - elapsed_)/b_fall_time;
  }
  else {
    wrench_.setZero();
    active_ = false;
  }
}

Vector6d KneeBucklingBehavior::externalWrench() const {
  return wrench_;
}
