#include "DragForce.h"
#include <GL/glut.h>

DragForce::DragForce(Particle *p) :
  m_p(p) {}

void DragForce::apply_force()
{
  m_p->m_Force -= k_d*m_p->m_Velocity;
}

void DragForce::draw()
{
  glBegin( GL_LINES );
  glColor3f(0.6, 0.7, 0.8);
  glVertex2f( m_p->m_Position[0], m_p->m_Position[1] );
  glEnd();
}