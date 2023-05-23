#pragma once
#include "linearSolver.h"

class Constraint
{
    public:
        virtual void draw() = 0;
        virtual double eval_C() = 0;
        virtual double eval_C_prime() = 0;
        virtual void compute_matrix_blocks(int i, SparseMatrix *J, SparseMatrix *J_prime) = 0;

};