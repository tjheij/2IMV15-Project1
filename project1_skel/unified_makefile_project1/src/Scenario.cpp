#include <vector>
#include "Particle.h"
#include "GravityForce.h"
#include "SpringForce.h"
#include "Force.h"
#include "CircularWireConstraint.h"
#include "RodConstraint.h"
#include "Constraint.h"
#include "Cloth.h"
#include <iostream>


void scenarioSpring(std::vector<Particle*> &particles, std::vector<Force*> &forces) {
    const float restDist = 0.2f; 
    
	const Vec2f center(0.0, 0.0);
    const Vec2f offset(0.1,0.0);
	particles.push_back(new Particle(center, 1.f));
	particles.push_back(new Particle(center + offset, 1.f));

	forces.push_back(new SpringForce(particles[0], particles[1], restDist, 1.f, 0.1f));
}

void scenarioGravity(std::vector<Particle*> &particles, std::vector<Force*> &forces) {
    const double dist = 0.2;
	const Vec2f center(0.0, 0.0);
	const Vec2f offset(dist, 0.0);

	// Create three particles, attach them to each other, then add a
	// circular wire constraint to the first.

	particles.push_back(new Particle(center + offset, 1.f));
	particles.push_back(new Particle(center + offset + offset, 1.f));
	particles.push_back(new Particle(center + offset + offset + offset, 1.f));

    //Add gravity
	for(Particle* p : particles){
		GravityForce* gravity = new GravityForce(p);
        forces.push_back(gravity);
	}
}

void scenarioConstraints(std::vector<Particle*> &particles, std::vector<Force*> &forces, std::vector<Constraint*> &constraints) {
	const Vec2f center(0.0, 0.0);
	const Vec2f p_1_offset(0.0, 0.0);
	const Vec2f circle_1_offset(-0.2, 0.0);
	const Vec2f p_2_offset(0.7, 0.0);
	const Vec2f circle_2_offset(0.8, 0.0);
	const Vec2f p_3_offset(0.1, 0.0);

	particles.push_back(new Particle(center + p_1_offset, 1.f));
	particles.push_back(new Particle(center + p_2_offset, 1.f));
	particles.push_back(new Particle(center + p_3_offset, 1.f));

    forces.push_back(new GravityForce(particles[0]));
	forces.push_back(new GravityForce(particles[1]));
	forces.push_back(new GravityForce(particles[2]));

	constraints.push_back(new CircularWireConstraint(particles[0], 0, circle_1_offset, 0.2f));
	constraints.push_back(new CircularWireConstraint(particles[1], 1, circle_2_offset, 0.1f));
	constraints.push_back(new RodConstraint(particles[0], 0, particles[2], 2, 0.1f));
}

void scenarioCloth(std::vector<Particle*> &particles, std::vector<Force*> &forces, std::vector<Constraint*> &constraints) {
	Cloth* cloth = new Cloth(4, 4, particles, forces, constraints);
	cloth->init(particles, forces, constraints);
}

void initScenario(std::vector<Particle*> &particles, std::vector<Force*> &forces, std::vector<Constraint*> &constraints, int scenarioId) {
    switch (scenarioId) {
        case 0:
            scenarioGravity(particles, forces);
			break;
		case 1:
			scenarioSpring(particles, forces);
			break;	
		case 2:
			scenarioConstraints(particles, forces, constraints);
			break;	
		case 3:
			scenarioCloth(particles, forces, constraints);
			break;
    }
}