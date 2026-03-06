#pragma once
#include "Behavior/Behavior.h"

class OutOfLineStSBackBehavior : public Behavior {
public:
  explicit OutOfLineStSBackBehavior(const std::string& n);

  std::string name() const override { return name_; }
  void update(double t, double dt) override;
  Vector6d externalWrench() const override;
  void trigger() override;
  void reset() override;
  bool isActive() const override { return active_; }

  // parameters
  double force_y;    // [N]
  double force_z;    // [N]
  double torque_x;   // [Nm] (pitch)
  double duration;   // [s]

private:
  std::string name_;
  bool active_;
  double elapsed_;
  Vector6d wrench_;
};
