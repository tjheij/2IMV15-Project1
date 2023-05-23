#include "RodConstraint.h"
#include <GL/glut.h>

RodConstraint::RodConstraint(Particle *p1, const int p1_index, Particle * p2, const int p2_index, const double dist) :
  m_p1(p1), m_p1_index(p1_index), m_p2(p2), m_p2_index(p2_index), m_dist(dist) {}

double RodConstraint::eval_C()
{
	Vec2f position = (m_p1->m_Position - m_p2->m_Position)/m_dist;
	return 0.5 * (position * position - 1);
}

double RodConstraint::eval_C_prime()
{
	Vec2f position = (m_p1->m_Position - m_p2->m_Position)/m_dist;
	return position * m_p1->m_Velocity;
}

void RodConstraint::compute_matrix_blocks(int i, SparseMatrix *J, SparseMatrix *J_prime)
{
  Vec2f position1 = (m_p1->m_Position - m_p2->m_Position)/m_dist;
	J->set(i, m_p1_index*2, position1[0]);
  J->set(i, m_p1_index*2+1, position1[1]);

  Vec2f position2 = (m_p2->m_Position - m_p1->m_Position)/m_dist;
  J->set(i, m_p2_index*2, position2[0]);
  J->set(i, m_p2_index*2+1, position2[1]);

	J_prime->set(i, m_p1_index*2, m_p1->m_Velocity[0]);
  J_prime->set(i, m_p1_index*2+1, m_p1->m_Velocity[1]);
  J_prime->set(i, m_p2_index*2, m_p2->m_Velocity[0]);
  J_prime->set(i, m_p2_index*2+1, m_p2->m_Velocity[1]);
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
