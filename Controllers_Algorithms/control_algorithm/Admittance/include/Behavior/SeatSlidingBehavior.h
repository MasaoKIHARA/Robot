#pragma once
#include "Behavior/Behavior.h"

class SeatSlidingBehavior : public Behavior {
public:
  explicit SeatSlidingBehavior(const std::string& n);

  std::string name() const override { return name_; }
  void update(double t, double dt) override;
  Vector6d externalWrench() const override;
  void trigger() override;
  void reset() override;
  bool isActive() const override { return active_; }

  // parameter
  double slide_force_x;   // [N]
  double duration;        // [s]
  bool only_when_lowZ;    // available only when z is low.
  double lowZ_threshold;  // [m]

private:
  std::string name_;
  bool active_;
  double elapsed_;
  Vector6d wrench_;
};