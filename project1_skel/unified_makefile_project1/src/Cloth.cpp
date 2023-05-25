#include "Cloth.h"
#include "Particle.h"
#include "Constraint.h"
#include "SpringForce.h"

#define ks_constraints 100.0f
#define kd_constraints 1.0f

Cloth::Cloth(int x, int y, std::vector<Particle *> &pVector, std::vector<Force *> &fVector,
              std::vector<Constraint *> &cVector) : width(x), height(y), pVector(pVector), fVector(fVector),
                                                    cVector(cVector)
{

}

void Cloth::init(std::vector<Particle*> &pVector, std::vector<Force*> &fVector, std::vector<Constraint*> &cVector)
{
    for (int i = 0; i < width; i++) {
        for (int j = 0; j < height; ++j) {
            Particle *p = new Particle(Vec2f(-0.5f+ (i * 0.1f), 0.5f + (j * -0.1f)), 1.f);
            pVector.push_back(p);
            GravityForce *gf = new GravityForce(p);
            fVector.push_back(gf);
        }
    }
    //spring force between particle and its right neighbor
    for (int i = 0; i < width - 1; i++) {
        for (int j = 0; j < height; ++j) {
            SpringForce *sf = new SpringForce(pVector[i + j * width], pVector[i + 1 + j * width], 0.1f, ks_constraints,kd_constraints);
            fVector.push_back(sf);
        }
    }
    //spring force between particle and its bottom neighbor
    for (int i = 0; i < width; i++) {
        for (int j = 0; j < height - 1; ++j) {
            SpringForce *sf = new SpringForce(pVector[i + j * width], pVector[i + (j + 1) * width], 0.1f, ks_constraints,kd_constraints);
            fVector.push_back(sf);
        }
    }
    //constraint force for the top row
    
    //constraint force for the top left corner
    Constraint *c1 = new CircularWireConstraint(pVector[0], 0, Vec2f(pVector[0]->m_Position[0],pVector[0]->m_Position[1]+0.1f), 0.1f);
    cVector.push_back(c1);
    //constraint force for the top right corner
    Constraint *c2 = new CircularWireConstraint(pVector[(width - 1) * height], (width-1)*height, Vec2f(pVector[(width-1)*height]->m_Position[0],pVector[(width-1)*height]->m_Position[1]+0.1f) , 0.1f);
    cVector.push_back(c2);
}
void Cloth::Cloth::reset()
{
}

void Cloth::draw()
{
}
