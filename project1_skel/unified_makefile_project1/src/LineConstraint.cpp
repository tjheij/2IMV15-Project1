#include "LineConstraint.h"
#include <GL/glut.h>

LineConstraint::LineConstraint(Particle *p, const int p_index, const double slope, const double intercept, const double multiplier) :
	m_p(p), m_p_index(p_index), m_slope(slope), m_intercept(intercept), m_multiplier(multiplier) {}

double LineConstraint::eval_C()
{
	//cy = ax + b
	return m_slope * m_p->m_Position[0] + m_intercept - m_multiplier * m_p->m_Position[1];
}

double LineConstraint::eval_C_prime()
{
	return Vec2f(m_slope, -m_multiplier) * m_p->m_Velocity;
}

void LineConstraint::compute_matrix_blocks(int i, SparseMatrix *J, SparseMatrix *J_prime)
{
	J->set(i, m_p_index*2, m_slope);
	J->set(i, m_p_index*2+1, -m_multiplier);
	J_prime->set(i, m_p_index*2, 0);
	J_prime->set(i, m_p_index*2+1, 0);
}

void LineConstraint::draw()
{
	int N = 64;
	glBegin( GL_LINES );
	if(m_multiplier != 0){
		glColor3f(0.6, 0.7, 0.8);
		glVertex2f(-0.5*N, (m_slope * -0.5*N + m_intercept) / m_multiplier);
		glColor3f(0.6, 0.7, 0.8);
		glVertex2f(0.5*N, (m_slope * 0.5*N + m_intercept) / m_multiplier);
	}else{
		glColor3f(0.6, 0.7, 0.8);
		glVertex2f(-m_intercept / m_slope, -0.5*N);
		glColor3f(0.6, 0.7, 0.8);
		glVertex2f(-m_intercept / m_slope, 0.5*N);
	}
	glEnd();
}
