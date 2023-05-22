#include "RodConstraint.h"
#include <GL/glut.h>

RodConstraint::RodConstraint(Particle *p1, Particle * p2, const double dist, const double ks, const double kd) :
  m_p1(p1), m_p2(p2), m_dist(dist), m_ks(ks), m_kd(kd) {}

void RodConstraint::apply_single_constraint(Particle *p1, Particle *p2, double dist)
{
  Vec2f position = (p1->m_Position - p2->m_Position)/dist;
	Vec2f velocity = p1->m_Velocity;
	Vec2f forces = p1->m_Force;

	float stiffnessPart = m_ks * dist*(position * position - 1);
	float dampingPart = m_kd * (position * velocity);
	
	float lambda = ((-forces) * position - p1->m_Mass * (velocity * velocity + dampingPart)) / (position * position);
	Vec2f constraint_force = lambda * position;
	p1->m_Force += constraint_force;
}

void RodConstraint::apply_constraint_force()
{
  apply_single_constraint(m_p1, m_p2, m_dist);
  apply_single_constraint(m_p2, m_p1, m_dist);
}

void RodConstraint::draw()
{
  glBegin( GL_LINES );
  glColor3f(0.8, 0.7, 0.6);
  glVertex2f( m_p1->m_Position[0], m_p1->m_Position[1] );
  glColor3f(0.8, 0.7, 0.6);
  glVertex2f( m_p2->m_Position[0], m_p2->m_Position[1] );
  glEnd();

}
