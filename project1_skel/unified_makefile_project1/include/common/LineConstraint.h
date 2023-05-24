#pragma once

#include "Particle.h"
#include "Constraint.h"

class LineConstraint : public Constraint {
 public:
  LineConstraint(Particle *p, const int p_index, const double a, const double b, const double c);

  double eval_C();
  double eval_C_prime();

  void compute_matrix_blocks(int i, SparseMatrix *J, SparseMatrix *J_prime);

  void draw();

 private:

  Particle * const m_p;
  int const m_p_index;
  double const m_a;
  double const m_b;
  double const m_c;
};