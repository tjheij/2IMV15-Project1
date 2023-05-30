#include "GravityForce.h"
#include <GL/glut.h>

GravityForce::GravityForce(Particle *p) :
  m_p(p) {}

void GravityForce::apply_force()
{
  m_p->m_Force[1] -= m_p->m_Mass*G;
  m_p->m_Force -= 0.4*m_p->m_Velocity;
}

void GravityForce::draw()
{
  glBegin( GL_LINES );
  glColor3f(0.6, 0.7, 0.8);
  glVertex2f( m_p->m_Position[0], m_p->m_Position[1] );
  glEnd();
}