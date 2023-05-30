#pragma once

#include "Force.h"
#include "Particle.h"
#include "linearSolver.h"

class SpringForceImplicit : public Force {
public:
    SpringForceImplicit(Particle *p1, const int p1_index, Particle * p2, const int p2_index, double dist, double ks, double kd);

    void apply_force();
    void compute_matrix_blocks(SparseMatrix *dfdx, SparseMatrix *dfdv);
    void compute_matrix_blocks_2(SparseMatrix *dfdx, SparseMatrix *dfdv);

    Vec2f calc_force_1(Vec2f pos1, Vec2f vel1, Vec2f pos2, Vec2f vel2);
    Vec2f calc_force_2(Vec2f pos1, Vec2f vel1, Vec2f pos2, Vec2f vel2);

    void draw();
private:
    Particle * const m_p1;   // particle 1
    int const m_p1_index;    // particle 1 jacobian index
    Particle * const m_p2;   // particle 2 
    int const m_p2_index;    // particle 2 jacobian index
    double const m_dist;     // rest length
    double const m_ks, m_kd; // spring strength constants
};
