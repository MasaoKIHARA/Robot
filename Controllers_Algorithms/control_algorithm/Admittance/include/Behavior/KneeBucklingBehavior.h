#pragma once
#include "Behavior/Behavior.h"

class KneeBucklingBehavior : public Behavior {
public:
  explicit KneeBucklingBehavior(const std::string& n);

  std::string name() const override { return name_; }
  void update(double t, double dt) override;
  Vector6d externalWrench() const override;
  double BScale() const override;
  void trigger() override;
  void reset() override;
  bool isActive() const override { return active_; }

  // parameter
  std::string mode;        // "impulse" or "reduce_B"
  double impulse_force_z;  // [N] 
  double impulse_duration; // [s]
  double b_min_scale;      // minimum scale when reduce_B(0..1)
  double b_fall_time;      // [s]
  double b_hold_time;      // [s]
  double b_rise_time;      // [s]

private:
  std::string name_;
  bool active_;
  double elapsed_;     // passed time from trigger [s]
  Vector6d wrench_;
  double b_scale_;
};