#include <vector>
#include "Particle.h"
#include "GravityForce.h"
#include "SpringForce.h"
#include "AngularSpringForce.h"
#include "Force.h"
#include "CircularWireConstraint.h"
#include "RodConstraint.h"
#include "LineConstraint.h"
#include "Constraint.h"
#include "Cloth.h"
#include "ClothImplicit.h"
#include "CollisionLine.h"
#include <iostream>


void scenarioSpring(std::vector<Particle*> &particles, std::vector<Force*> &forces) {
    const float restDist = 0.2f; 
    
	const Vec2f center(0.0, 0.0);
    const Vec2f offset(0.1,0.0);
	particles.push_back(new Particle(center, 1.f));
	particles.push_back(new Particle(center + offset, 1.f));

	forces.push_back(new SpringForce(particles[0], particles[1], restDist, 1.f, 0.1f));
}

void scenarioGravity(std::vector<Particle*> &particles, std::vector<Force*> &forces, std::vector<CollisionLine*> &colliders) {
    const double dist = 0.4;
	const Vec2f center(0.0, 0.3);
	const Vec2f offset(dist, 0.0);

	// Create three particles, attach them to each other, then add a
	// circular wire constraint to the first.

	particles.push_back(new Particle(center - offset, 2.f));
	particles.push_back(new Particle(center, 5.f));
	particles.push_back(new Particle(center + offset, 10.f));

    //Add gravity
	for(Particle* p : particles){
		GravityForce* gravity = new GravityForce(p);
        forces.push_back(gravity);
	}

	colliders.push_back(new CollisionLine(Vec2f(0.0,-0.5), Vec2f(0.0,1.0), 0.015f));
}

void scenarioConstraints(std::vector<Particle*> &particles, std::vector<Force*> &forces, std::vector<Constraint*> &constraints) {
	const Vec2f center(0.0, 0.0);
	const Vec2f p_1_offset(0.0, 0.0);
	const Vec2f circle_1_offset(-0.25, 0.0);
	const Vec2f p_2_offset(0.6, 0.0);
	const Vec2f circle_2_offset(0.7, 0.0);
	const Vec2f p_3_offset(0.4, 0.3);
	const Vec2f p_4_offset(0.2, 0.0);

	particles.push_back(new Particle(center + p_1_offset, 1.f));
	particles.push_back(new Particle(center + p_2_offset, 1.f));
	particles.push_back(new Particle(center + p_3_offset, 1.f));
	particles.push_back(new Particle(center + p_4_offset, 2.f));

    forces.push_back(new GravityForce(particles[0]));
	forces.push_back(new GravityForce(particles[1]));
	forces.push_back(new GravityForce(particles[2]));
	forces.push_back(new GravityForce(particles[3]));

	constraints.push_back(new CircularWireConstraint(particles[0], 0, circle_1_offset, 0.25f));
	constraints.push_back(new CircularWireConstraint(particles[1], 1, circle_2_offset, 0.1f));
	constraints.push_back(new RodConstraint(particles[0], 0, particles[2], 2, 0.5f));
	constraints.push_back(new LineConstraint(particles[2], 2, 0.0f, 0.3f, 1.0f));
	constraints.push_back(new RodConstraint(particles[1], 1, particles[3], 3, 0.4f));
}

void scenarioCloth(std::vector<Particle*> &particles, std::vector<Force*> &forces, std::vector<Constraint*> &constraints, int type) {
	Cloth* cloth = new Cloth(10, 10, particles, forces, constraints);
	cloth->init(particles, forces, constraints, type);
}

void scenarioClothImplicit(std::vector<Particle*> &particles, std::vector<Force*> &forces, std::vector<Constraint*> &constraints, int type) {
	ClothImplicit* cloth = new ClothImplicit(20, 20, particles, forces, constraints);
	cloth->init(particles, forces, constraints, type);
}

void scenarioAngularSpring(std::vector<Particle*> &particles, std::vector<Force*> &forces, std::vector<Constraint*> &constraints) {
    const float restAngle = 2;
	const float restLength = 0.1;
    
	const Vec2f p1(0.1, 0.0);
    const Vec2f p2(0.0, 0.0);
    const Vec2f p3(0.0, 0.1);
	particles.push_back(new Particle(p1, 1.f));
	particles.push_back(new Particle(p2, 1.f));
	particles.push_back(new Particle(p3, 1.f));

	constraints.push_back(new RodConstraint(particles[0], 0, particles[1], 1, 0.1));
	constraints.push_back(new RodConstraint(particles[2], 2, particles[1], 1, 0.1));

	forces.push_back(new AngularSpringForce(particles[0], particles[1], particles[2], restAngle, 0.1f, 0.1f));
}

void initScenario(std::vector<Particle*> &particles, std::vector<Force*> &forces, std::vector<Constraint*> &constraints, std::vector<CollisionLine*> &colliders, int scenarioId) {
    switch (scenarioId) {
        case 0:
            scenarioGravity(particles, forces, colliders);
			break;
		case 1:
			scenarioSpring(particles, forces);
			break;	
		case 2:
			scenarioConstraints(particles, forces, constraints);
			break;	
		case 3:
			scenarioCloth(particles, forces, constraints, 0);
			break;
		case 4:
			scenarioCloth(particles, forces, constraints, 1);
			break;
		case 5:
			scenarioClothImplicit(particles, forces, constraints, 0);
			break;
		case 6:
			scenarioAngularSpring(particles, forces, constraints);
			break;
    }
}