#include "Particle.h"
#include "Force.h"
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
void midpoint(std::vector<Particle*> &pVector, float dt) {
	for(Particle* p : pVector) {
		// 1st Order
		p->m_Position += 0.5f * dt * p->m_Velocity;
		p->m_Velocity += 0.5f * dt * (p->m_Force / p->m_Mass);

		// 2nd Order
		p->m_Position += dt * p->m_Velocity;
		p->m_Velocity += dt * (p->m_Force / p->m_Mass);
	}
}

void rungeKutta(std::vector<Particle*> &pVector, float dt) {
	for(Particle* p : pVector) {
		// 2nd Order
		p->m_Position += 0.5f * dt * p->m_Velocity;
		p->m_Velocity += 0.5f * dt * (p->m_Force / p->m_Mass);

		// 3rd Order
		p->m_Position += 0.5f * dt * p->m_Velocity;
		p->m_Velocity += 0.5f * dt * (p->m_Force / p->m_Mass);

		// 4th Order
		p->m_Position += dt * p->m_Velocity;
		p->m_Velocity += dt * (p->m_Force / p->m_Mass);

		// Vec2f old_P = p->m_Position;
		// Vec2f old_V = p->m_Velocity;

		// Vec2f k1_P = dt * p->m_Velocity;
		// Vec2f k1_V = dt * (p->m_Force / p->m_Mass);
		// p->set_state(old_P+k1_P/2, old_V+k1_V/2);

		// Vec2f k2_P = dt * p->m_Velocity;
		// Vec2f k2_V = dt * (p->m_Force / p->m_Mass);
		// p->set_state(old_P+k2_P/2, old_V+k2_V/2);

		// Vec2f k3_P = dt * p->m_Velocity;
		// Vec2f k3_V = dt * (p->m_Force / p->m_Mass);
		// p->set_state(old_P+k3_P, old_V+k3_V);

		// Vec2f k4_P = dt * p->m_Velocity;
		// Vec2f k4_V = dt * (p->m_Force / p->m_Mass);

		// Vec2f new_P = old_P + (1.0f/6.0f)*k1_P + (1.0f/3.0f)*k2_P + (1.0f/3.0f)*k3_P + (1.0f/6.0f)*k4_P;
		// Vec2f new_V = old_V + (1.0f/6.0f)*k1_V + (1.0f/3.0f)*k2_V + (1.0f/3.0f)*k3_V + (1.0f/6.0f)*k4_V;

		// p->set_state(new_P, new_V);


	}

}
void integrate(std::vector<Particle*> &pVector, float dt, int scheme) {
	// if (scheme == 1) {
	// 	eulerSemiImplicit(pVector, dt);
	// } else if (scheme == 2) {
	// 	midpoint(pVector, dt);
	// } else {
	// 	rungeKutta(pVector, dt);
	// }
	eulerSemiImplicit(pVector, dt);
	
}

void simulation_step(std::vector<Particle*> &pVector, std::vector<Force*> &fVector, float dt, int scheme) {
	clear_forces(pVector);
	calculate_forces(fVector);
	integrate(pVector, dt, scheme);
}

