#pragma once

#include "Particle.h"
#include "Constraint.h"
#include "CircularWireConstraint.h"
#include "RodConstraint.h"
#include "Force.h"
#include "GravityForce.h"
#include "DragForce.h"
#include "SpringForce.h"
#include "CollisionLine.h"
#include <math.h>

class Cloth
{
public:
    Cloth(int x, int y, std::vector<Particle*> &pVector, std::vector<Force*> &fVector, std::vector<Constraint*> &cVector);
    void init(std::vector<Particle *> &pVector, std::vector<Force *> &fVector, std::vector<Constraint *> &cVector, int type, std::vector<CollisionLine *> &colliders);  
    void structral_spring(std::vector<Particle *> &pVector, std::vector<Force *> &fVector);
    void shear_spring(std::vector<Particle *> &pVector, std::vector<Force *> &fVector);
    void flexion_spring(std::vector<Particle *> &pVector, std::vector<Force *> &fVector);
    void constraints(std::vector<Particle *> &pVector, std::vector<Constraint *> &cVector, int type);
    ~Cloth();
    void reset();
    void draw();
    std::vector<Force*> fVector;
    std::vector<Constraint*> cVector;
    std::vector<Particle*> pVector;
private:
    int width;
    int height;
    float deltaX;
    float deltaY;

};