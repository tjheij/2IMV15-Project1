#include "Particle.h"

#include <vector>

#include "Force.h"

#define DAMP 0.98f
#define RAND (((rand()%2000)/1000.f)-1.f)

void clear_forces( std::vector<Particle*> &pVector )
{
	for (Particle* p : pVector) {
		p->m_Force = Vec2(0.0, 0.0);
	}
}

extern void calculate_forces( std::vector<Force*> &fVector )
{
	for (Force* f : fVector) {
		f->apply_force();
	}
}

void integrate( std::vector<Particle*> &pVector, float dt )
{
	int ii, size = pVector.size();
	
	int doExplicit = 0;

	for(ii=0; ii<size; ii++)
	{
		if (doExplicit) {
			// Explicit Euler
			pVector[ii]->m_Position += dt*pVector[ii]->m_Velocity;
			pVector[ii]->m_Velocity += dt*pVector[ii]->m_Force / pVector[ii]->m_Mass; 
		} else {
			// Semi-implicit Euler
			pVector[ii]->m_Velocity += dt*pVector[ii]->m_Force / pVector[ii]->m_Mass; 
			pVector[ii]->m_Position += dt*pVector[ii]->m_Velocity;
		}
	}
} 

void simulation_step( std::vector<Particle*> &pVector, std::vector<Force*> &fVector, float dt )
{
	clear_forces(pVector);
	calculate_forces(fVector);
	integrate(pVector, dt);
}

