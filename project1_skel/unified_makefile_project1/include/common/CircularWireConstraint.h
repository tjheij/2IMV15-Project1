#pragma once

#include "Particle.h"
#include "Constraint.h"

class CircularWireConstraint : public Constraint {
 public:
  CircularWireConstraint(Particle *p, const int p_index, const Vec2f & center, const double radius);

  double eval_C();
  double eval_C_prime();

  void compute_matrix_blocks(int i, SparseMatrix *J, SparseMatrix *J_prime);

  void draw();

 private:

  Particle * const m_p;
  int const m_p_index;
  Vec2f const m_center;
  double const m_radius;
};
