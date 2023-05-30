#pragma once

#include "Force.h"
#include "Particle.h"

class AngularSpringForce : public Force {
public:
    AngularSpringForce(Particle *p1, Particle *p2, Particle *p3, double angle, double ks, double kd);

    void apply_force();
    double fC(Vec2f x, Vec2f y, Vec2f z);

    void draw();
 private:
    Particle * const m_p1;   // particle 1
    Particle * const m_p2;   // particle 2 
    Particle * const m_p3;   // particle 2 
    double const m_angle;     // rest length
    double const m_ks, m_kd; // spring strength constants
};
