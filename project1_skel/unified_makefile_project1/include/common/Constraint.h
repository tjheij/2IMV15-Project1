#pragma once

class Constraint
{
    public:
        virtual void apply_constraint_force() = 0;
        virtual void draw() = 0;

        struct matrix_block{
            int i;
            int j;
            int ilength;
            int jlength;
            float* data;
        };

};