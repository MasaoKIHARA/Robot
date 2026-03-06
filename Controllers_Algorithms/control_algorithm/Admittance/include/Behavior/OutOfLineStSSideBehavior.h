#pragma once
#include "Behavior/Behavior.h"

class OutOfLineStSSideBehavior : public Behavior {
public:
  explicit OutOfLineStSSideBehavior(const std::string& n);

  std::string name() const override { return name_; }
  void update(double t, double dt) override;
  Vector6d externalWrench() const override;
  void trigger() override;
  void reset() override;
  bool isActive() const override { return active_; }

  // parameters
  double force_x;    // [N]
  double force_y;    // [N]
  double force_z;    // [N]
  double torque_y;   // [Nm] (roll)
  double duration;   // [s]

private:
  std::string name_;
  bool active_;
  double elapsed_;
  Vector6d wrench_;
};
