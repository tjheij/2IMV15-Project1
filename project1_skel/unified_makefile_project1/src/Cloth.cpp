#include "Cloth.h"
#include "Particle.h"
#include "Constraint.h"
#include "SpringForce.h"
#include "CircularWireConstraint.h"
#include "LineConstraint.h"

#define ks_constraints 500.0f
#define kd_constraints 5.f

Cloth::Cloth(int x, int y, std::vector<Particle *> &pVector, std::vector<Force *> &fVector,
              std::vector<Constraint *> &cVector) : width(x), height(y), pVector(pVector), fVector(fVector),
                                                    cVector(cVector)
{
    deltaX = 1.0f/width;
    deltaY = 1.0f/height;
}

void Cloth::init(std::vector<Particle*> &pVector, std::vector<Force*> &fVector, std::vector<Constraint*> &cVector, int type)
{
    //create particles
    for (int i = 0; i < width; i++) {
        for (int j = 0; j < height; ++j) {
            Particle *p = new Particle(Vec2f(-0.5f+ (i * deltaX), 0.5f + (j * -deltaY)), 1.f);
            pVector.push_back(p);
            //add gravity force to the bottom particles
            GravityForce *gf = new GravityForce(p);
            fVector.push_back(gf);
        }
    }
    
    structral_spring(pVector, fVector);
    shear_spring(pVector, fVector);   
    flexion_spring(pVector, fVector);

    constraints(pVector, cVector, type);
    
    
 
  
}

void Cloth::structral_spring(std::vector<Particle *> &pVector, std::vector<Force *> &fVector)
{
    //spring force between particle and its bottom neighbor
    for (int i,j = 0; i < pVector.size(); i++) {
        SpringForce *sf = new SpringForce(pVector[i], pVector[i+1], deltaY, ks_constraints,kd_constraints);
        fVector.push_back(sf);
        j++;
        if (j == height-1) {
            j = 0;
            i++;
        }    
    }
    //spring force between particle and its right neighbor
    for (int i = 0; i<pVector.size()-height;i++) {
        SpringForce *sf = new SpringForce(pVector[i], pVector[i+height], deltaX, ks_constraints,kd_constraints);
        fVector.push_back(sf);
    }
}
void Cloth::shear_spring(std::vector<Particle *> &pVector, std::vector<Force *> &fVector)
{
    float diagonalDistance = sqrt(pow(deltaX,2)+pow(deltaY,2));
    // spring force diagonal
    for (int i,j= 0; i<pVector.size()-height;i++) {
        SpringForce *sf = new SpringForce(pVector[i], pVector[i+height+1], diagonalDistance, ks_constraints,kd_constraints);
        fVector.push_back(sf);
        j++;
        if (j == height-1) {
            j = 0;
            i++;
        }
    }
    //spring force other diagonal
    for (int i,j= 0; i<pVector.size()-height;i++) {
        if (j == height-1) {
            j = 0;
            continue;
        }
        SpringForce *sf = new SpringForce(pVector[i+1], pVector[i+height], diagonalDistance, ks_constraints,kd_constraints);
        fVector.push_back(sf);
        j++;
    }
}

void Cloth::flexion_spring(std::vector<Particle *> &pVector, std::vector<Force *> &fVector)
{
    //spring force between particle and its bottom neighbor
    for (int i,j = 0; i < pVector.size(); i++) {
        SpringForce *sf = new SpringForce(pVector[i], pVector[i+2], 2*deltaY, ks_constraints,kd_constraints);
        fVector.push_back(sf);
        j++;
        if (j == height-2) {
            j = 0;
            i += 2;
        }    
    }
    //spring force between particle and its right neighbor
    for (int i = 0; i<pVector.size()-(height*2);i++) {
        SpringForce *sf = new SpringForce(pVector[i], pVector[i+(height*2)], 2*deltaX, ks_constraints,kd_constraints);
        fVector.push_back(sf);
    }
}
void Cloth::constraints(std::vector<Particle *> &pVector, std::vector<Constraint *> &cVector, int type)
{
    if (type == 1) {
        int j = 0;
        for (int i = 0; i < pVector.size(); i+=height) {
            //line constraint for top row
            if (j % 2 == 0) {
                Constraint *c = new LineConstraint(pVector[i], i, 0.0f, 0.5f, 1.0f);
                cVector.push_back(c);
            }
            j++;
        }
    } else {
        //constraint force for the top left corner
        Constraint *c1 = new CircularWireConstraint(pVector[0], 0, Vec2f(pVector[0]->m_Position[0],pVector[0]->m_Position[1]+0.1f), 0.1f);
        cVector.push_back(c1);
        //constraint force for the top right corner
        Constraint *c2 = new CircularWireConstraint(pVector[(width - 1) * height], (width-1)*height, Vec2f(pVector[(width-1)*height]->m_Position[0],pVector[(width-1)*height]->m_Position[1]+0.1f) , 0.1f);
        cVector.push_back(c2);
    }
  
}
void Cloth::reset()
{
}

void Cloth::draw()
{
}
