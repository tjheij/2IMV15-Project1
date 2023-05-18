#include "Particle.h"
#include "Force.h"
#include <vector>

void clear_forces(std::vector<Particle*> &pVector) {
	for (Particle* p : pVector) {
		p->m_Force = Vec2(0.0, 0.0);
	}
}

extern void calculate_forces( std::vector<Force*> &fVector) {
	for (Force* f : fVector) {
		f->apply_force();
	}
}

void eulerExplicit(std::vector<Particle*> &pVector, float dt) {
	for(Particle* p : pVector) {
		p->m_Position += dt*p->m_Velocity;
		p->m_Velocity += dt*p->m_Force / p->m_Mass; 
	}
}

void eulerSemiImplicit(std::vector<Particle*> &pVector, float dt) {
	for(Particle* p : pVector) {
		p->m_Velocity += dt*p->m_Force / p->m_Mass; 
		p->m_Position += dt*p->m_Velocity;
	}
}

void integrate(std::vector<Particle*> &pVector, float dt) {
	eulerSemiImplicit(pVector, dt);
}

void simulation_step(std::vector<Particle*> &pVector, std::vector<Force*> &fVector, float dt) {
	clear_forces(pVector);
	calculate_forces(fVector);
	integrate(pVector, dt);
}

