#include "SpringForce.h"
#include <GL/glut.h>

#include <gfx/vec2.h>

SpringForce::SpringForce(Particle *p1, Particle * p2, double dist, double ks, double kd) :
  m_p1(p1), m_p2(p2), m_dist(dist), m_ks(ks), m_kd(kd) {}

void SpringForce::apply_force()
{
  Vec2f delta_x = m_p1->m_Position - m_p2->m_Position;
  Vec2f delta_v = m_p1->m_Velocity - m_p2->m_Velocity;

  Vec2f stiffnessPart = m_ks * (norm(delta_x) - m_dist);
  Vec2f dampingPart = m_kd * (delta_v * delta_x) / norm(delta_x);

  Vec2f f1 = -(stiffnessPart + dampingPart) * (delta_x / norm(delta_x));
  Vec2f f2 = -f1;

  m_p1->m_Force += f1;
  m_p2->m_Force += f2;
}

void SpringForce::draw()
{
  glBegin( GL_LINES );
  glColor3f(0.6, 0.7, 0.8);
  glVertex2f( m_p1->m_Position[0], m_p1->m_Position[1] );
  glColor3f(0.6, 0.7, 0.8);
  glVertex2f( m_p2->m_Position[0], m_p2->m_Position[1] );
  glEnd();
}
