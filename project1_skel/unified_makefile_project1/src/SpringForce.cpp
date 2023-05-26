#include "SpringForce.h"
#include <GL/glut.h>

#include <gfx/vec2.h>

SpringForce::SpringForce(Particle *p1, Particle * p2, double dist, double ks, double kd) :
    m_p1(p1), m_p2(p2), m_dist(dist), m_ks(ks), m_kd(kd) {}

void SpringForce::apply_force() {
    Vec2f delta_x = m_p1->m_Position - m_p2->m_Position;
    Vec2f delta_v = m_p1->m_Velocity - m_p2->m_Velocity;

    Vec2f stiffnessPart = m_ks * (norm(delta_x) - m_dist);
    Vec2f dampingPart = m_kd * (delta_v * delta_x) / norm(delta_x);

    Vec2f f1 = -(stiffnessPart + dampingPart) * (delta_x / norm(delta_x));
    Vec2f f2 = -f1;

    m_p1->m_Force += f1;
    m_p2->m_Force += f2;
}

void SpringForce::add_jacobians(SparseMatrix *dfdx, SparseMatrix *dfdv) {
    std::size_t p1 = m_p1->m_Index;
    std::size_t p2 = m_p2->m_Index;

    Vec2f delta_x = m_p1->m_Position - m_p2->m_Position;
    Vec2f delta_v = m_p1->m_Velocity - m_p2->m_Velocity;

    double norm_dx = norm(delta_x);
    double norm_dx3 = norm_dx * norm_dx * norm_dx;

    double dfdx00 = 0.0;
    double dfdx01 = 0.0;
    double dfdx10 = 0.0;
    double dfdx11 = 0.0;

    double dfdv00 = 0.0;
    double dfdv01 = 0.0;
    double dfdv10 = 0.0;
    double dfdv11 = 0.0;

    {
        double alpha = m_ks * (norm_dx - m_dist) + m_kd * (delta_v * delta_x) / norm_dx;

        double dv_d00 = (1 / norm_dx - delta_x[0] * delta_x[0]) / norm_dx3;
        double dv_d01 = (1 / norm_dx - delta_x[1] * delta_x[1]) / norm_dx3;
        double dv_d10 = -((delta_x[0] * delta_x[1]) / norm_dx3);
        double dv_d11 = dv_d10;

        double da0_d00 = ks * delta_x[0] / norm_dx;
        double da0_d01 = ks * delta_x[1] / norm_dx;
        double da0_d10 = da0_d00;
        double da0_d11 = da0_d01;

        double da1_d00 = kd * (delta_v[0] / norm_dx - delta_x[0] * (delta_v * delta_x) / norm_dx3);
        double da1_d01 = kd * (delta_v[1] / norm_dx - delta_x[0] * (delta_v * delta_x) / norm_dx3);
        double da1_d10 = da1_d00;
        double da1_d11 = da1_d01;

        dfdx00 = alpha * dv_d00 + delta_dx[0] / norm_dx * (da0_d00 + da1_d00);
        dfdx01 = alpha * dv_d01 + delta_dx[0] / norm_dx * (da0_d01 + da1_d01);
        dfdx10 = alpha * dv_d10 + delta_dx[1] / norm_dx * (da0_d10 + da1_d10);
        dfdx11 = alpha * dv_d11 + delta_dx[1] / norm_dx * (da0_d11 + da1_d11)
    }

    {
        double da_d00 = m_kd * delta_x[0] / norm_dx;
        double da_d01 = m_kd * delta_x[1] / norm_dx;
        double da_d10 = da_d00;
        double da_d11 = da_d01;

        dfdv00 = delta_x[0] / norm_dx * da_d00;
        dfdv01 = delta_x[0] / norm_dx * da_d01;
        dfdv10 = delta_x[1] / norm_dx * da_d10;
        dfdv11 = delta_x[1] / norm_dx * da_d11;
    }

    dfdx->add(p1 * 2 + 0, p1 * 2 + 0, dfdx00);
    dfdx->add(p1 * 2 + 0, p1 * 2 + 1, dfdx01);
    dfdx->add(p1 * 2 + 1, p1 * 2 + 0, dfdx10);
    dfdx->add(p1 * 2 + 1, p1 * 2 + 1, dfdx11);

    dfdx->add(p1 * 2 + 0, p2 * 2 + 0, -dfdx00);
    dfdx->add(p1 * 2 + 0, p2 * 2 + 1, -dfdx01);
    dfdx->add(p1 * 2 + 1, p2 * 2 + 0, -dfdx10);
    dfdx->add(p1 * 2 + 1, p2 * 2 + 1, -dfdx11);
    
    dfdx->add(p2 * 2 + 0, p1 * 2 + 0, -dfdx00);
    dfdx->add(p2 * 2 + 0, p1 * 2 + 1, -dfdx01);
    dfdx->add(p2 * 2 + 1, p1 * 2 + 0, -dfdx10);
    dfdx->add(p2 * 2 + 1, p1 * 2 + 1, -dfdx11);
    
    dfdx->add(p2 * 2 + 0, p2 * 2 + 0, dfdx00);
    dfdx->add(p2 * 2 + 0, p2 * 2 + 1, dfdx01);
    dfdx->add(p2 * 2 + 1, p2 * 2 + 0, dfdx10);
    dfdx->add(p2 * 2 + 1, p2 * 2 + 1, dfdx11);


    dfdv->add(p1 * 2 + 0, p1 * 2 + 0, dfdv00);
    dfdv->add(p1 * 2 + 0, p1 * 2 + 1, dfdv01);
    dfdv->add(p1 * 2 + 1, p1 * 2 + 0, dfdv10);
    dfdv->add(p1 * 2 + 1, p1 * 2 + 1, dfdv11);

    dfdv->add(p1 * 2 + 0, p2 * 2 + 0, -dfdv00);
    dfdv->add(p1 * 2 + 0, p2 * 2 + 1, -dfdv01);
    dfdv->add(p1 * 2 + 1, p2 * 2 + 0, -dfdv10);
    dfdv->add(p1 * 2 + 1, p2 * 2 + 1, -dfdv11);

    dfdv->add(p2 * 2 + 0, p1 * 2 + 0, -dfdv00);
    dfdv->add(p2 * 2 + 0, p1 * 2 + 1, -dfdv01);
    dfdv->add(p2 * 2 + 1, p1 * 2 + 0, -dfdv10);
    dfdv->add(p2 * 2 + 1, p1 * 2 + 1, -dfdv11);

    dfdv->add(p2 * 2 + 0, p2 * 2 + 0, dfdv00);
    dfdv->add(p2 * 2 + 0, p2 * 2 + 1, dfdv01);
    dfdv->add(p2 * 2 + 1, p2 * 2 + 0, dfdv10);
    dfdv->add(p2 * 2 + 1, p2 * 2 + 1, dfdv11);
}

void SpringForce::draw() {
    glBegin( GL_LINES );
    glColor3f(0.6, 0.7, 0.8);
    glVertex2f( m_p1->m_Position[0], m_p1->m_Position[1] );
    glColor3f(0.6, 0.7, 0.8);
    glVertex2f( m_p2->m_Position[0], m_p2->m_Position[1] );
    glEnd();
}
