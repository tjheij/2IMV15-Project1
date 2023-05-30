#include "SpringForceImplicit.h"
#include <GL/glut.h>

#include <gfx/vec2.h>

SpringForceImplicit::SpringForceImplicit(Particle *p1, const int p1_index, Particle * p2, const int p2_index, double dist, double ks, double kd) :
    m_p1(p1), m_p1_index(p1_index), m_p2(p2), m_p2_index(p2_index), m_dist(dist), m_ks(ks), m_kd(kd) {}

void SpringForceImplicit::apply_force() {
    Vec2f delta_x = m_p1->m_Position - m_p2->m_Position;
    Vec2f delta_v = m_p1->m_Velocity - m_p2->m_Velocity;
    float length = norm(delta_x);

    // Vec2f stiffnessPart = m_ks * (length - m_dist);
    // Vec2f dampingPart = m_kd * ((delta_v * delta_x) / length);
    // Vec2f f1 = -(stiffnessPart + dampingPart) * (delta_x / length);

    Vec2f f1 = -(m_ks*(length - m_dist) + m_kd*((delta_v * delta_x) / length)) * (delta_x / length);
    Vec2f f2 = -f1;

    m_p1->m_Force += f1;
    m_p2->m_Force += f2;
}

/**
void SpringForceImplicit::compute_matrix_blocks(SparseMatrix *dfdx, SparseMatrix *dfdv) {
    std::size_t p1 = m_p1_index;
    std::size_t p2 = m_p2_index;

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

    double ks = m_ks;
    double kd = m_kd;

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

        dfdx00 = alpha * dv_d00 + delta_x[0] / norm_dx * (da0_d00 + da1_d00);
        dfdx01 = alpha * dv_d01 + delta_x[0] / norm_dx * (da0_d01 + da1_d01);
        dfdx10 = alpha * dv_d10 + delta_x[1] / norm_dx * (da0_d10 + da1_d10);
        dfdx11 = alpha * dv_d11 + delta_x[1] / norm_dx * (da0_d11 + da1_d11);
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

    dfdv->add(p1 * 2 + 0, p1 * 2 + 0, dfdv00);
    dfdv->add(p1 * 2 + 0, p1 * 2 + 1, dfdv01);
    dfdv->add(p1 * 2 + 1, p1 * 2 + 0, dfdv10);
    dfdv->add(p1 * 2 + 1, p1 * 2 + 1, dfdv11);

    dfdx->add(p1 * 2 + 0, p2 * 2 + 0, -dfdx00);
    dfdx->add(p1 * 2 + 0, p2 * 2 + 1, -dfdx01);
    dfdx->add(p1 * 2 + 1, p2 * 2 + 0, -dfdx10);
    dfdx->add(p1 * 2 + 1, p2 * 2 + 1, -dfdx11);

    dfdv->add(p1 * 2 + 0, p2 * 2 + 0, -dfdv00);
    dfdv->add(p1 * 2 + 0, p2 * 2 + 1, -dfdv01);
    dfdv->add(p1 * 2 + 1, p2 * 2 + 0, -dfdv10);
    dfdv->add(p1 * 2 + 1, p2 * 2 + 1, -dfdv11);
    

    dfdx->add(p2 * 2 + 0, p1 * 2 + 0, -dfdx00);
    dfdx->add(p2 * 2 + 0, p1 * 2 + 1, -dfdx01);
    dfdx->add(p2 * 2 + 1, p1 * 2 + 0, -dfdx10);
    dfdx->add(p2 * 2 + 1, p1 * 2 + 1, -dfdx11);

    dfdv->add(p2 * 2 + 0, p1 * 2 + 0, -dfdv00);
    dfdv->add(p2 * 2 + 0, p1 * 2 + 1, -dfdv01);
    dfdv->add(p2 * 2 + 1, p1 * 2 + 0, -dfdv10);
    dfdv->add(p2 * 2 + 1, p1 * 2 + 1, -dfdv11);
    

    dfdx->add(p2 * 2 + 0, p2 * 2 + 0, dfdx00);
    dfdx->add(p2 * 2 + 0, p2 * 2 + 1, dfdx01);
    dfdx->add(p2 * 2 + 1, p2 * 2 + 0, dfdx10);
    dfdx->add(p2 * 2 + 1, p2 * 2 + 1, dfdx11);

    dfdv->add(p2 * 2 + 0, p2 * 2 + 0, dfdv00);
    dfdv->add(p2 * 2 + 0, p2 * 2 + 1, dfdv01);
    dfdv->add(p2 * 2 + 1, p2 * 2 + 0, dfdv10);
    dfdv->add(p2 * 2 + 1, p2 * 2 + 1, dfdv11);
}*/

Vec2f SpringForceImplicit::calc_force_1(Vec2f pos1, Vec2f vel1, Vec2f pos2, Vec2f vel2) {
    Vec2f delta_x = pos1 - pos2;
    Vec2f delta_v = vel1 - vel2;
    float length = norm(delta_x);

    // Vec2f stiffnessPart = m_ks * (length - m_dist);
    // Vec2f dampingPart = m_kd * ((delta_v * delta_x) / length);
    // Vec2f f1 = -(stiffnessPart + dampingPart) * (delta_x / length);

    Vec2f f1 = -(m_ks*(length - m_dist) + m_kd*((delta_v * delta_x) / length)) * (delta_x / length);
    Vec2f f2 = -f1;

    return f1;
}

Vec2f SpringForceImplicit::calc_force_2(Vec2f pos1, Vec2f vel1, Vec2f pos2, Vec2f vel2) {
    Vec2f delta_x = pos1 - pos2;
    Vec2f delta_v = vel1 - vel2;
    float length = norm(delta_x);

    // Vec2f stiffnessPart = m_ks * (length - m_dist);
    // Vec2f dampingPart = m_kd * ((delta_v * delta_x) / length);
    // Vec2f f1 = -(stiffnessPart + dampingPart) * (delta_x / length);

    Vec2f f1 = -(m_ks*(length - m_dist) + m_kd*((delta_v * delta_x) / length)) * (delta_x / length);
    Vec2f f2 = -f1;

    return f2;
}

void SpringForceImplicit::compute_matrix_blocks(SparseMatrix *dfdx, SparseMatrix *dfdv) {
    std::size_t p1 = m_p1_index;
    std::size_t p2 = m_p2_index;

    Vec2f pos1 = m_p1->m_Position;
    Vec2f pos2 = m_p2->m_Position;
    Vec2f vel1 = m_p1->m_Velocity;
    Vec2f vel2 = m_p2->m_Velocity;

    Vec2f f0_1 = calc_force_1(pos1, vel1, pos2, vel2);
    Vec2f f0_2 = calc_force_2(pos1, vel1, pos2, vel2);

    double dh = 0.01;
    Vec2f h1 = Vec2f(dh, 0);
    Vec2f h2 = Vec2f(0, dh);

    dfdx->add(p1 * 2 + 0, p1 * 2 + 0, (calc_force_1(pos1 + h1, vel1, pos2, vel2)[0] - f0_1[0]) / dh);
    dfdx->add(p1 * 2 + 0, p1 * 2 + 1, (calc_force_1(pos1 + h2, vel1, pos2, vel2)[0] - f0_1[0]) / dh);
    dfdx->add(p1 * 2 + 1, p1 * 2 + 0, (calc_force_1(pos1 + h1, vel1, pos2, vel2)[1] - f0_1[1]) / dh);
    dfdx->add(p1 * 2 + 1, p1 * 2 + 1, (calc_force_1(pos1 + h2, vel1, pos2, vel2)[1] - f0_1[1]) / dh);

    dfdv->add(p1 * 2 + 0, p1 * 2 + 0, (calc_force_1(pos1, vel1 + h1, pos2, vel2)[0] - f0_1[0]) / dh);
    dfdv->add(p1 * 2 + 0, p1 * 2 + 1, (calc_force_1(pos1, vel1 + h2, pos2, vel2)[0] - f0_1[0]) / dh);
    dfdv->add(p1 * 2 + 1, p1 * 2 + 0, (calc_force_1(pos1, vel1 + h1, pos2, vel2)[1] - f0_1[1]) / dh);
    dfdv->add(p1 * 2 + 1, p1 * 2 + 1, (calc_force_1(pos1, vel1 + h2, pos2, vel2)[1] - f0_1[1]) / dh);

    dfdx->add(p1 * 2 + 0, p2 * 2 + 0, (calc_force_1(pos1, vel1, pos2 + h1, vel2)[0] - f0_1[0]) / dh);
    dfdx->add(p1 * 2 + 0, p2 * 2 + 1, (calc_force_1(pos1, vel1, pos2 + h2, vel2)[0] - f0_1[0]) / dh);
    dfdx->add(p1 * 2 + 1, p2 * 2 + 0, (calc_force_1(pos1, vel1, pos2 + h1, vel2)[1] - f0_1[1]) / dh);
    dfdx->add(p1 * 2 + 1, p2 * 2 + 1, (calc_force_1(pos1, vel1, pos2 + h2, vel2)[1] - f0_1[1]) / dh);

    dfdv->add(p1 * 2 + 0, p2 * 2 + 0, (calc_force_1(pos1, vel1, pos2, vel2 + h1)[0] - f0_1[0]) / dh);
    dfdv->add(p1 * 2 + 0, p2 * 2 + 1, (calc_force_1(pos1, vel1, pos2, vel2 + h2)[0] - f0_1[0]) / dh);
    dfdv->add(p1 * 2 + 1, p2 * 2 + 0, (calc_force_1(pos1, vel1, pos2, vel2 + h1)[1] - f0_1[1]) / dh);
    dfdv->add(p1 * 2 + 1, p2 * 2 + 1, (calc_force_1(pos1, vel1, pos2, vel2 + h2)[1] - f0_1[1]) / dh);
    

    dfdx->add(p2 * 2 + 0, p1 * 2 + 0, (calc_force_2(pos1 + h1, vel1, pos2, vel2)[0] - f0_2[0]) / dh);
    dfdx->add(p2 * 2 + 0, p1 * 2 + 1, (calc_force_2(pos1 + h2, vel1, pos2, vel2)[0] - f0_2[0]) / dh);
    dfdx->add(p2 * 2 + 1, p1 * 2 + 0, (calc_force_2(pos1 + h1, vel1, pos2, vel2)[1] - f0_2[1]) / dh);
    dfdx->add(p2 * 2 + 1, p1 * 2 + 1, (calc_force_2(pos1 + h2, vel1, pos2, vel2)[1] - f0_2[1]) / dh);

    dfdv->add(p2 * 2 + 0, p1 * 2 + 0, (calc_force_2(pos1, vel1 + h1, pos2, vel2)[0] - f0_2[0]) / dh);
    dfdv->add(p2 * 2 + 0, p1 * 2 + 1, (calc_force_2(pos1, vel1 + h2, pos2, vel2)[0] - f0_2[0]) / dh);
    dfdv->add(p2 * 2 + 1, p1 * 2 + 0, (calc_force_2(pos1, vel1 + h1, pos2, vel2)[1] - f0_2[1]) / dh);
    dfdv->add(p2 * 2 + 1, p1 * 2 + 1, (calc_force_2(pos1, vel1 + h2, pos2, vel2)[1] - f0_2[1]) / dh);

    dfdx->add(p2 * 2 + 0, p2 * 2 + 0, (calc_force_2(pos1, vel1, pos2 + h1, vel2)[0] - f0_2[0]) / dh);
    dfdx->add(p2 * 2 + 0, p2 * 2 + 1, (calc_force_2(pos1, vel1, pos2 + h2, vel2)[0] - f0_2[0]) / dh);
    dfdx->add(p2 * 2 + 1, p2 * 2 + 0, (calc_force_2(pos1, vel1, pos2 + h1, vel2)[1] - f0_2[1]) / dh);
    dfdx->add(p2 * 2 + 1, p2 * 2 + 1, (calc_force_2(pos1, vel1, pos2 + h2, vel2)[1] - f0_2[1]) / dh);

    dfdv->add(p2 * 2 + 0, p2 * 2 + 0, (calc_force_2(pos1, vel1, pos2, vel2 + h1)[0] - f0_2[0]) / dh);
    dfdv->add(p2 * 2 + 0, p2 * 2 + 1, (calc_force_2(pos1, vel1, pos2, vel2 + h2)[0] - f0_2[0]) / dh);
    dfdv->add(p2 * 2 + 1, p2 * 2 + 0, (calc_force_2(pos1, vel1, pos2, vel2 + h1)[1] - f0_2[1]) / dh);
    dfdv->add(p2 * 2 + 1, p2 * 2 + 1, (calc_force_2(pos1, vel1, pos2, vel2 + h2)[1] - f0_2[1]) / dh);
}

void SpringForceImplicit::draw() {
    glBegin( GL_LINES );
    glColor3f(0.6, 0.7, 0.8);
    glVertex2f( m_p1->m_Position[0], m_p1->m_Position[1] );
    glColor3f(0.6, 0.7, 0.8);
    glVertex2f( m_p2->m_Position[0], m_p2->m_Position[1] );
    glEnd();
}
