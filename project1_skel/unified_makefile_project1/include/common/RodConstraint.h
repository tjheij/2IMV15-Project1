#pragma once

#include "Particle.h"
#include "Constraint.h"

class RodConstraint : public Constraint {
 public:
  RodConstraint(Particle *p1, const int p1_index, Particle * p2, const int p2_index, const double dist);

  double eval_C();
  double eval_C_prime();

  void compute_matrix_blocks(int i, SparseMatrix *J, SparseMatrix *J_prime);

  void draw();

 private:

  Particle * const m_p1;
  int const m_p1_index;
  Particle * const m_p2;
  int const m_p2_index;
  double const m_dist;
};
