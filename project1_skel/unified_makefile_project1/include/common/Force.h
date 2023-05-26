#pragma once

class Force
{
    public:
        virtual void apply_force() = 0;
        virtual void add_jacobians(SparseMatrix *dfdx, SparseMatrix *dfdv) = 0;
        virtual void draw() = 0;
};