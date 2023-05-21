#pragma once

#include "Particle.h"
#include "Constraint.h"

class RodConstraint : public Constraint {
 public:
  RodConstraint(Particle *p1, Particle * p2, const double dist, const double ks, const double kd);

  void apply_constraint_force();
  void apply_single_constraint(Particle *p1, Particle *p2, double dist);

  void draw();

 private:

  Particle * const m_p1;
  Particle * const m_p2;
  double const m_dist;
  const double m_ks;
  const double m_kd;
};
