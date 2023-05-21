#pragma once

#include "Particle.h"
#include "Constraint.h"

class CircularWireConstraint : public Constraint {
 public:
  CircularWireConstraint(Particle *p, const Vec2f & center, const double radius, const double ks, const double kd);

  void apply_constraint_force();

  void draw();

 private:

  Particle * const m_p;
  Vec2f const m_center;
  double const m_radius;
  double const m_ks;
  double const m_kd;
};
