#include "Particle.h"
#include "Force.h"
#include "Constraint.h"
#include <vector>

void clear_forces(std::vector<Particle*> &pVector) {
	for (Particle* p : pVector) {
		p->m_Force = Vec2(0.0, 0.0);
	}
}

void calculate_forces( std::vector<Force*> &fVector) {
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

void eulerExplicit(std::vector<Particle*> &pVector, std::vector<Force*> &fVector, std::vector<Constraint*> &cVector, float dt) {
	clear_forces(pVector);
	calculate_forces(fVector);
	calculate_constraint_forces(cVector);
	for(Particle* p : pVector) {
		p->m_Position += dt*p->m_Velocity;
		p->m_Velocity += dt*p->m_Force / p->m_Mass; 
	}
}

void eulerSemiImplicit(std::vector<Particle*> &pVector, std::vector<Force*> &fVector, std::vector<Constraint*> &cVector, float dt) {
	clear_forces(pVector);
	calculate_forces(fVector);
	calculate_constraint_forces(cVector);
	for(Particle* p : pVector) {
		p->m_Velocity += dt*p->m_Force / p->m_Mass; 
		p->m_Position += dt*p->m_Velocity;
	}
}
void midpoint(std::vector<Particle*> &pVector, std::vector<Force*> &fVector, std::vector<Constraint*> &cVector, float dt) {
	
	std::vector<Vec2f> start_positions = std::vector<Vec2f>();
	std::vector<Vec2f> start_velocities = std::vector<Vec2f>();

	//add start positions and velocities to vectors
	for(Particle* p : pVector) {
		start_positions.push_back(p->m_Position);
		start_velocities.push_back(p->m_Velocity);
	}

	
	clear_forces(pVector);
	calculate_forces(fVector);
	calculate_constraint_forces(cVector);
	//do first order midpoint step
	for(Particle* p : pVector) {
		p->m_Velocity += dt * (p->m_Force / p->m_Mass);
		p->m_Position += 0.5f * dt * p->m_Velocity;
	}

	clear_forces(pVector);
	calculate_forces(fVector);
	calculate_constraint_forces(cVector);

	//do second order midpoint step
	for(int i = 0; i < pVector.size(); i++) {
		pVector[i]->m_Velocity = start_velocities[i] + dt * (pVector[i]->m_Force / pVector[i]->m_Mass);
		pVector[i]->m_Position = start_positions[i] + dt * pVector[i]->m_Velocity;
	}
}

void rungeKutta(std::vector<Particle*> &pVector, std::vector<Force*> &fVector, std::vector<Constraint*> &cVector, float dt) {
	
	std::vector<Vec2f> start_positions = std::vector<Vec2f>();
	std::vector<Vec2f> start_velocities = std::vector<Vec2f>();
	std::vector<Vec2f> k1 = std::vector<Vec2f>();
	std::vector<Vec2f> k2 = std::vector<Vec2f>();
	std::vector<Vec2f> k3 = std::vector<Vec2f>();
	std::vector<Vec2f> k4 = std::vector<Vec2f>();
	std::vector<Vec2f> v1 = std::vector<Vec2f>();
	std::vector<Vec2f> v2 = std::vector<Vec2f>();
	std::vector<Vec2f> v3 = std::vector<Vec2f>();
	std::vector<Vec2f> v4 = std::vector<Vec2f>();
	
	//add start positions and velocities to vectors
	for (Particle* p : pVector) {
		start_positions.push_back(p->m_Position);
		start_velocities.push_back(p->m_Velocity);
		k1.push_back(Vec2f(0.0, 0.0));	
		k2.push_back(Vec2f(0.0, 0.0));
		k3.push_back(Vec2f(0.0, 0.0));
		k4.push_back(Vec2f(0.0, 0.0));
		v1.push_back(Vec2f(0.0, 0.0));
		v2.push_back(Vec2f(0.0, 0.0));
		v3.push_back(Vec2f(0.0, 0.0));
		v4.push_back(Vec2f(0.0, 0.0));
	}

	clear_forces(pVector);
	calculate_forces(fVector);
	calculate_constraint_forces(cVector);

	//do first order runge kutta step
	for (int i = 0; i < pVector.size(); i++) {
		k1[i] = dt * (pVector[i]->m_Force / pVector[i]->m_Mass);
		v1[i] = dt * pVector[i]->m_Velocity;

		pVector[i]->m_Velocity += 0.5f  * k1[i]; 
		pVector[i]->m_Position += 0.5f  * v1[i];
	}

	clear_forces(pVector);
	calculate_forces(fVector);
	calculate_constraint_forces(cVector);

	//do second order runge kutta step
	for (int i = 0; i < pVector.size(); i++) {
		k2[i] = dt * (pVector[i]->m_Force / pVector[i]->m_Mass);
		v2[i] = dt * pVector[i]->m_Velocity;

		pVector[i]->m_Velocity += 0.5f * k2[i];
		pVector[i]->m_Position += 0.5f * v2[i];
	}
	
	clear_forces(pVector);
	calculate_forces(fVector);
	calculate_constraint_forces(cVector);

	//do third order runge kutta step
	for (int i = 0; i < pVector.size(); i++) {
		k3[i] = dt * (pVector[i]->m_Force / pVector[i]->m_Mass);
		v3[i] = dt * pVector[i]->m_Velocity;

		pVector[i]->m_Velocity += k3[i];
		pVector[i]->m_Position += v3[i];
	}
	clear_forces(pVector);
	calculate_forces(fVector);
	calculate_constraint_forces(cVector);

	//do fourth order runge kutta step
	for (int i = 0; i < pVector.size(); i++) {
		k4[i] = dt * (pVector[i]->m_Force / pVector[i]->m_Mass);
		v4[i] = dt * pVector[i]->m_Velocity;

		pVector[i]->m_Velocity = start_velocities[i] + (1.0f / 6.0f) * (k1[i] + 2.0f * k2[i] + 2.0f * k3[i] + k4[i]);
		pVector[i]->m_Position = start_positions[i] + (1.0f / 6.0f) * (v1[i] + 2.0f * v2[i] + 2.0f * v3[i] + v4[i]);
	}

}


void simulation_step(std::vector<Particle*> &pVector, std::vector<Force*> &fVector, std::vector<Constraint*> &cVector, float dt, int scheme) {
	switch (scheme) {
		case 0:
		default:
			eulerSemiImplicit(pVector, fVector, cVector, dt);
			break;
		case 1: 
			midpoint(pVector, fVector, cVector, dt);
			break;
		case 2:
			rungeKutta(pVector, fVector, cVector, dt);
			break;
	}
}

