#include "CircularWireConstraint.h"
#include <GL/glut.h>

#define PI 3.1415926535897932384626433832795

CircularWireConstraint::CircularWireConstraint(Particle *p, const int p_index, const Vec2f & center, const double radius) :
	m_p(p), m_p_index(p_index), m_center(center), m_radius(radius) {}

double CircularWireConstraint::eval_C()
{
	Vec2f position = (m_p->m_Position - m_center)/m_radius;
	return 0.5 * (position * position - 1);
}

double CircularWireConstraint::eval_C_prime()
{
	Vec2f position = (m_p->m_Position - m_center)/m_radius;
	return position * m_p->m_Velocity;
}

void CircularWireConstraint::compute_matrix_blocks(int i, SparseMatrix *J, SparseMatrix *J_prime)
{
	Vec2f position = (m_p->m_Position - m_center)/m_radius;
	J->set(i, m_p_index*2, position[0]);
	J->set(i, m_p_index*2+1, position[1]);
	J_prime->set(i, m_p_index*2, m_p->m_Velocity[0]);
	J_prime->set(i, m_p_index*2+1, m_p->m_Velocity[1]);
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
