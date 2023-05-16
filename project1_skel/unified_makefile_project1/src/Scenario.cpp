#include <vector>
#include "Particle.h"
#include "GravityForce.h"
#include "SpringForce.h"
#include "Force.h"

#include <iostream>


void scenarioSpring(std::vector<Particle*> &particles, std::vector<Force*> &forces) {
    const float dist = 0.3f;
    const float restDist = 0.2f; 
    
	const Vec2f center(0.0, 0.0);
	const Vec2f offset(dist, 0.0);
    
	particles.push_back(new Particle(center));
	particles.push_back(new Particle(center + offset));

    forces.push_back(new SpringForce(particles[0], particles[1], restDist, 1.0f, 0.0f));
}

void scenarioGravity(std::vector<Particle*> &particles, std::vector<Force*> &forces) {
    const double dist = 0.2;
	const Vec2f center(0.0, 0.0);
	const Vec2f offset(dist, 0.0);

	// Create three particles, attach them to each other, then add a
	// circular wire constraint to the first.

	particles.push_back(new Particle(center + offset));
	particles.push_back(new Particle(center + offset + offset));
	particles.push_back(new Particle(center + offset + offset + offset));

    //Add gravity
	for(Particle* p : particles){
		GravityForce* gravity = new GravityForce(p);
        forces.push_back(gravity);
	}
}

void initScenario(std::vector<Particle*> &particles, std::vector<Force*> &forces, int scenarioId) {
    switch (scenarioId) {
        case 0:
            scenarioGravity(particles, forces);
		case 1:
			scenarioSpring(particles, forces);	
    }
}