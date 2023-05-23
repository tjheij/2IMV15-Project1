#include "CircularWireConstraint.h"
#include <GL/glut.h>

#define PI 3.1415926535897932384626433832795

CircularWireConstraint::CircularWireConstraint(Particle *p, const int p_index, const Vec2f & center, const double radius, const double ks, const double kd) :
	m_p(p), m_p_index(p_index), m_center(center), m_radius(radius), m_ks(ks), m_kd(kd) {}

void CircularWireConstraint::apply_constraint_force() 
{
	Vec2f position = (m_p->m_Position - m_center)/m_radius;
	Vec2f velocity = m_p->m_Velocity;
	Vec2f forces = m_p->m_Force;

	float stiffnessPart = m_ks * 0.5 *(position * position - 1);
	float dampingPart = m_kd * (position * velocity);
	
	float lambda = ((-forces) * position - m_p->m_Mass * (velocity * velocity + stiffnessPart + dampingPart)) / (position * position);
	Vec2f constraint_force = lambda * position;
	m_p->m_Force += constraint_force;
}

float CircularWireConstraint::eval_C()
{
	Vec2f position = (m_p->m_Position - m_center)/m_radius;
	return 0.5 * (position * position - 1);
}

float CircularWireConstraint::eval_C_prime()
{
	Vec2f position = (m_p->m_Position - m_center)/m_radius;
	return position * m_p->m_Velocity;
}

double[] eval_J(int n){
	double column[2*n] = {0};
	column[p_index]
}

void CircularWireConstraint::compute_matrix_blocks(std::vector<matrix_block> J, std::vector<matrix_block> J_prime, int i, int j, int ilength, int jlength)
{
	matrix_block blockJ = {i, p_index, ilength, jlength, (m_p->m_Position - m_center)/m_radius};
	J.push_back(blockJ);
	matrix_block blockJ_prime = {i, p_index, ilength, jlength, m_p->m_Velocity};
	J_prime.push_back(blockJ_prime);
}

static void draw_circle(const Vec2f & vect, float radius)
{
	glBegin(GL_LINE_LOOP);
	glColor3f(0.0,1.0,0.0); 
	for (int i=0; i<360; i=i+18)
	{
		float degInRad = i*PI/180;
		glVertex2f(vect[0]+cos(degInRad)*radius,vect[1]+sin(degInRad)*radius);
	}
	glEnd();
}

void CircularWireConstraint::draw()
{
	draw_circle(m_center, m_radius);
}
