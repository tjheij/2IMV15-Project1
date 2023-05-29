#pragma once

#include "Force.h"
#include "Particle.h"

#define G 0.1f

class GravityForce : public Force {
 public:
  GravityForce(Particle *p);

  void apply_force();

  void draw();

 private:

  Particle * const m_p;   // particle 
};
