#pragma once
#include "Behavior/Behavior.h"

class StSBehavior : public Behavior {
public:
  explicit StSBehavior(const std::string& n);

  std::string name() const override { return name_; }
  void update(double t, double dt) override;
  Vector6d externalWrench() const override;
  void trigger() override;
  void reset() override;
  bool isActive() const override { return active_; }

  // parameters
  double force_y;   // [N]
  double force_z;   // [N]
  double duration;  // [s]

private:
  std::string name_;
  bool active_;
  double elapsed_;
  Vector6d wrench_;
};
