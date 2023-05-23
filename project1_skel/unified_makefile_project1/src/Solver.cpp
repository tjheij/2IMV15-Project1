#include "Particle.h"
#include "Force.h"
#include "Constraint.h"
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

void calculate_constraint_forces(std::vector<Particle*> &pVector, std::vector<Constraint*> &cVector) {
	int m = cVector.size();
	int n = pVector.size();

	double C[m] = {0};
	double C_prime[m] = {0};
	double J[m][n] = {0};

	for (int i = 0; i < m; i++) {
		//Get c, c', J, J'
		C[i] = cVector[i]->eval_C();
		C_prime[i] = cVector[i]->eval_C_prime();
		cVector[i]->compute_matrix_blocks();
	}

	for (matrix_block block : constraints_J){
		block.i
	}

	//Calculate lhs matrix
	implicitMatrixWithTrans JWJ_T;

	//Calculate rhs vector
	double rhs[m];

	//Linsolve lambda
	double lambda[m];
	ConjGrad(m, JWJ_T, lambda, rhs);

	//Calculate new forces
	double Q[n] = {0}; //J_T * lambda

	//Add forces
	for(int i = 0; i < n; i++){
		pVector[i]->m_Force += Q[i];
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

void simulation_step(std::vector<Particle*> &pVector, std::vector<Force*> &fVector, std::vector<Constraint*> &cVector, float dt) {
	clear_forces(pVector);
	calculate_forces(fVector);
	calculate_constraint_forces(cVector);
	integrate(pVector, dt);
}

