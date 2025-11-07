#include "Behavior/KneeBucklingBehavior.h"
#include <algorithm>

KneeBucklingBehavior::KneeBucklingBehavior(const std::string& n)
: mode("impulse"), impulse_force_z(-80.0), impulse_duration(0.03),
  b_min_scale(0.2), b_fall_time(0.05), b_hold_time(0.2), b_rise_time(0.2),
  name_(n), active_(false), elapsed_(0.0), b_scale_(1.0)
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
  if (!active_) { wrench_.setZero(); b_scale_ = 1.0; return; }
  elapsed_ += dt;

  if (mode == "impulse") {
    if (elapsed_ <= impulse_duration) {
      wrench_.setZero();
      wrench_(2) = impulse_force_z; // z方向
    } else {
      wrench_.setZero();
      active_ = false;
    }
    b_scale_ = 1.0;
  } else { // reduce_B
    double t = elapsed_;
    if (t <= b_fall_time) {
      double r = (b_fall_time <= 1e-6) ? 1.0 : (t / b_fall_time);
      b_scale_ = 1.0 + (b_min_scale - 1.0) * r;
    } else if (t <= b_fall_time + b_hold_time) {
      b_scale_ = b_min_scale;
    } else if (t <= b_fall_time + b_hold_time + b_rise_time) {
      double r = (t - b_fall_time - b_hold_time) / std::max(1e-6, b_rise_time);
      b_scale_ = b_min_scale + (1.0 - b_min_scale) * r;
    } else {
      b_scale_ = 1.0;
      active_ = false;
    }
    wrench_.setZero();
  }
}

Vector6d KneeBucklingBehavior::externalWrench() const {
  return wrench_;
}

double KneeBucklingBehavior::BScale() const {
  return b_scale_;
}