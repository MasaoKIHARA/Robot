#pragma once
#include <string>
#include <Eigen/Core>

using namespace Eigen;

class Behavior {
public:
  virtual ~Behavior() {}
  virtual std::string name() const = 0;

  virtual void update(double t, double dt) = 0;

  virtual Vector6d externalWrench() const { return Vector6d::Zero(); }

  // patient model B_ Scaling
  virtual double BScale() const { return 1.0; }

  virtual void trigger() = 0;
  virtual void reset() = 0;

  virtual bool isActive() const = 0;
};