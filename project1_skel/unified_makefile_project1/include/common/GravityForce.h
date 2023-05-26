#pragma once

#include "Force.h"
#include "Particle.h"

#define G 1.0f

class GravityForce : public Force {
 public:
  GravityForce(Particle *p);

  void apply_force();
  void add_jacobians(SparseMatrix *dfdx, SparseMatrix *dfdv);
  void draw();

 private:

  Particle * const m_p;   // particle 
};
