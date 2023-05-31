#pragma once

#include "Force.h"
#include "Particle.h"

#define k_d 0.4f

class DragForce : public Force {
 public:
  DragForce(Particle *p);

  void apply_force();

  void draw();

 private:

  Particle * const m_p;   // particle 
};
