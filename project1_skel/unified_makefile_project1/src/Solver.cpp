#include "Particle.h"
#include "Force.h"
#include "Constraint.h"
#include <vector>

#define ks_constraints 100.0f
#define kd_constraints 1.0f

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
	int n = pVector.size() * 2; //2 dimensions per particle

	double C[m] = {0};
	double C_prime[m] = {0};

	double q_prime[n] = {0};
	double Q[n] = {0};

	double W[n] = {0};
	SparseMatrix J = SparseMatrix(m,n);
	SparseMatrix J_prime = SparseMatrix(m,n);

	for (int i = 0; i < m; i++) {
		//Get c, c', J, J'
		C[i] = cVector[i]->eval_C();
		C_prime[i] = cVector[i]->eval_C_prime();
		cVector[i]->compute_matrix_blocks(i, &J, &J_prime);
	}

	for (int i = 0; i < pVector.size(); i++) {
		q_prime[i*2] = pVector[i]->m_Velocity[0];
		q_prime[i*2+1] = pVector[i]->m_Velocity[1];
		Q[i*2] = pVector[i]->m_Force[0];
		Q[i*2+1] = pVector[i]->m_Force[1];
		W[i*2] = W[i*2+1] = 1.0f/pVector[i]->m_Mass;
	}

	//Calculate rhs vector
	double rhs[m] = {0};
	double temp[m] = {0};
	//-J'q'^T
	J_prime.matVecMult(q_prime, temp);
	vecDiffEqual(m, rhs, temp);
	//-JWQ^T
	vecTimesElementWise(n, Q, W);
	J.matVecMult(Q, temp);
	vecDiffEqual(m, rhs, temp);
	//-ksC
	vecTimesScalar(m, C, ks_constraints);
	vecDiffEqual(m, rhs, C);
	//-kdC'
	vecTimesScalar(m, C_prime, kd_constraints);
	vecDiffEqual(m, rhs, C_prime);

	//Linsolve lambda
	double lambda[m] = {0};
	int steps = 100;
	VectorConjGrad(m, n, &J, W, lambda, rhs, 0.001f, &steps);

	//Calculate new forces
	double Q_hat[n] = {0};
	J.matTransVecMult(lambda, Q_hat); //J_T * lambda

	//Add forces
	for(int i = 0; i < pVector.size(); i++){
		pVector[i]->m_Force += Vec2f(Q_hat[i*2], Q_hat[i*2+1]);
	}
}

void eulerExplicit(std::vector<Particle*> &pVector, std::vector<Force*> &fVector, std::vector<Constraint*> &cVector, float dt) {
	clear_forces(pVector);
	calculate_forces(fVector);
	calculate_constraint_forces(pVector, cVector);
	for(Particle* p : pVector) {
		p->m_Position += dt*p->m_Velocity;
		p->m_Velocity += dt*p->m_Force / p->m_Mass; 
	}
}

void eulerSemiImplicit(std::vector<Particle*> &pVector, std::vector<Force*> &fVector, std::vector<Constraint*> &cVector, float dt) {
	clear_forces(pVector);
	calculate_forces(fVector);
	calculate_constraint_forces(pVector, cVector);
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
	calculate_constraint_forces(pVector, cVector);
	//do first order midpoint step
	for(Particle* p : pVector) {
		p->m_Velocity += dt * (p->m_Force / p->m_Mass);
		p->m_Position += 0.5f * dt * p->m_Velocity;
	}

	clear_forces(pVector);
	calculate_forces(fVector);
	calculate_constraint_forces(pVector, cVector);

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
	calculate_constraint_forces(pVector, cVector);

	//do first order runge kutta step
	for (int i = 0; i < pVector.size(); i++) {
		k1[i] = dt * (pVector[i]->m_Force / pVector[i]->m_Mass);
		v1[i] = dt * pVector[i]->m_Velocity;

		pVector[i]->m_Velocity += 0.5f  * k1[i]; 
		pVector[i]->m_Position += 0.5f  * v1[i];
	}

	clear_forces(pVector);
	calculate_forces(fVector);
	calculate_constraint_forces(pVector, cVector);

	//do second order runge kutta step
	for (int i = 0; i < pVector.size(); i++) {
		k2[i] = dt * (pVector[i]->m_Force / pVector[i]->m_Mass);
		v2[i] = dt * pVector[i]->m_Velocity;

		pVector[i]->m_Velocity += 0.5f * k2[i];
		pVector[i]->m_Position += 0.5f * v2[i];
	}
	
	clear_forces(pVector);
	calculate_forces(fVector);
	calculate_constraint_forces(pVector, cVector);

	//do third order runge kutta step
	for (int i = 0; i < pVector.size(); i++) {
		k3[i] = dt * (pVector[i]->m_Force / pVector[i]->m_Mass);
		v3[i] = dt * pVector[i]->m_Velocity;

		pVector[i]->m_Velocity += k3[i];
		pVector[i]->m_Position += v3[i];
	}
	clear_forces(pVector);
	calculate_forces(fVector);
	calculate_constraint_forces(pVector, cVector);

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

