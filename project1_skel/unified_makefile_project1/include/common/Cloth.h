#pragma once

#include "Particle.h"
#include "Constraint.h"
#include "CircularWireConstraint.h"
#include "RodConstraint.h"
#include "Force.h"
#include "GravityForce.h"
#include "SpringForce.h"
#include <math.h>

class Cloth
{
public:
    Cloth(int x, int y, std::vector<Particle*> &pVector, std::vector<Force*> &fVector, std::vector<Constraint*> &cVector);
    void init(std::vector<Particle *> &pVector, std::vector<Force *> &fVector, std::vector<Constraint *> &cVector);
    ~Cloth();
    void reset();
    void draw();
    std::vector<Force*> fVector;
    std::vector<Constraint*> cVector;
    std::vector<Particle*> pVector;
private:
    int width;
    int height;

};