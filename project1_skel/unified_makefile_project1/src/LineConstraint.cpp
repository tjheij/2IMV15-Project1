#include "LineConstraint.h"
#include <GL/glut.h>

LineConstraint::LineConstraint(Particle *p, const int p_index, const double a, const double b, const double c) :
	m_p(p), m_p_index(p_index), m_a(a), m_b(b), m_c(c) {}

double LineConstraint::eval_C()
{
	//cy = ax + b
	return m_a * m_p->m_Position[0] + m_b - m_c * m_p->m_Position[1];
}

double LineConstraint::eval_C_prime()
{
	return Vec2f(m_a, -m_c) * m_p->m_Velocity;
}

void LineConstraint::compute_matrix_blocks(int i, SparseMatrix *J, SparseMatrix *J_prime)
{
	J->set(i, m_p_index*2, m_a);
	J->set(i, m_p_index*2+1, -m_c);
	J_prime->set(i, m_p_index*2, 0);
	J_prime->set(i, m_p_index*2+1, 0);
}

void LineConstraint::draw()
{
	int N = 64;
	glBegin( GL_LINES );
	glColor3f(0.6, 0.7, 0.8);
	glVertex2f(-0.5*N, (m_a*-0.5*N + m_b)/m_c);
	glColor3f(0.6, 0.7, 0.8);
	glVertex2f(0.5*N, (m_a*0.5*N + m_b)/m_c);
	glEnd();
}
