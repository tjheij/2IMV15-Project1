#include "AngularSpringForce.h"
#include <GL/glut.h>
#include <cmath>

#include <iostream>

#include <gfx/vec2.h>

AngularSpringForce::AngularSpringForce(Particle *p1, Particle *p2, Particle *p3, double angle, double ks, double kd) :
  m_p1(p1), m_p2(p2), m_p3(p3), m_angle(angle), m_ks(ks), m_kd(kd) {}

double AngularSpringForce::fC(Vec2f x, Vec2f y, Vec2f z) {
    return std::acos(((x - y) * (z - y)) / (norm(x - y) * norm(z - y))) - m_angle;
}

double safe_acos(double value) {
    if (value <= -1.0) {
        return 3.141592653;
    } else if (value>= 1.0) {
        return 0;
    } else {
        return std::acos(value);
    }
}


void AngularSpringForce::apply_force() {
    Vec2f x = m_p1->m_Position;
    Vec2f y = m_p2->m_Position;
    Vec2f z = m_p3->m_Position;

    Vec2f u = m_p1->m_Velocity;
    Vec2f v = m_p2->m_Velocity;
    Vec2f w = m_p3->m_Velocity;

    double cosTheta = (x - y) * (z - y) / (norm(x - y) * norm(z - y));
    double C = safe_acos(cosTheta) - m_angle;
    double damping = 0.1;

    double dCdtT1 = ((u - v) * (z - y)) + ((x - y) * (w - v));
    double dCdtT2 = -((x - y) * (z - y)) * (((x - y) * (u - v)) / norm2(x - y) + ((z - y) * (w - v)) / norm2(z - y));

    double dCdt = 1 / (norm(x - y) * norm(z - y)) * (dCdtT1 + dCdtT2);

    Vec2f dCdxT1 = (1 / (norm(x - y) * norm(z - y))) * (z - y);
    Vec2f dCdxT2 = -cosTheta * (1 / norm2(x - y)) * (x - y);

    Vec2f dCdzT1 = (1 / (norm(x - y) * norm(x - y))) * (x - y);
    Vec2f dCdzT2 = -cosTheta * (1 / norm2(z - y)) * (z - y);

    Vec2f dCdx = -1 / std::sqrt(damping + 1 - cosTheta * cosTheta) * (dCdxT1 + dCdxT2);
    Vec2f dCdy = 1 / std::sqrt(damping + 1 - cosTheta * cosTheta) * (dCdxT1 + dCdzT1 + dCdxT2 + dCdzT2);
    Vec2f dCdz = -1 / std::sqrt(damping + 1 - cosTheta * cosTheta) * (dCdzT1 + dCdzT2);

    double magnitude = -(m_ks * C - m_kd * dCdt);
    Vec2f f1 = magnitude * dCdx;
    Vec2f f2 = magnitude * dCdy;
    Vec2f f3 = magnitude * dCdz;

//    std::cout << cosTheta << " | " << C << " " << dCdt << std::endl;

    m_p1->m_Force += f1;
    m_p2->m_Force += f2;
    m_p3->m_Force += f3;
}

void AngularSpringForce::draw()
{

}
