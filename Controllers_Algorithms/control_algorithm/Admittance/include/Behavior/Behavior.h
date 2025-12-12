#pragma once
#include <string>
#include <Eigen/Core>

using namespace Eigen;

typedef Matrix<double, 6, 1> Vector6d;

class Behavior {
public:
  virtual ~Behavior() {}
  virtual std::string name() const = 0;

  virtual void update(double t, double dt) = 0;

  virtual Vector6d externalWrench() const { return Vector6d::Zero(); }

  virtual void trigger() = 0;
  virtual void reset() = 0;

  virtual bool isActive() const = 0;
};