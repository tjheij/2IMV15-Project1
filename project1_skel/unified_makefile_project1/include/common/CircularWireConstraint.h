#pragma once

#include "Particle.h"
#include "Constraint.h"

class CircularWireConstraint : public Constraint {
 public:
  CircularWireConstraint(Particle *p, const int p_index, const Vec2f & center, const double radius, const double ks, const double kd);

  void apply_constraint_force();

  float eval_C();
  float eval_C_prime();

  void compute_matrix_blocks(std::vector<matrix_block> J, std::vector<matrix_block> J_prime, int i, int j, int ilength, int jlength);

  void draw();

 private:

  Particle * const m_p;
  int const m_p_index;
  Vec2f const m_center;
  double const m_radius;
  double const m_ks;
  double const m_kd;
};
