#include "CollisionLine.h"
#include "Particle.h"
#include <GL/glut.h>

CollisionLine::CollisionLine(const Vec2f position, const Vec2f normal, const float e) : m_Position(position), m_Normal(normal), m_E(e) {}

void CollisionLine::collisionWith(Particle* p)
{
    float r = 0.9f;
    float distance = (p->m_Position - m_Position) * m_Normal;

    //Is in wall, or close to wall and heading in wall
    if (distance < 0 || (distance < m_E && m_Normal * p->m_Velocity < 0)) {
        Vec2f v_normal = (m_Normal * p->m_Velocity) * m_Normal;
        Vec2f v_tangent = p->m_Velocity - v_normal;
        p->m_Velocity = v_tangent - r * v_normal;
    }
}

void CollisionLine::collisionForceWith(Particle* p)
{
    float r = 0.9f;
    float distance = (p->m_Position - m_Position) * m_Normal;
    
    //Is in wall, or close to wall and heading in wall
    if (distance < 0 || (distance < m_E && m_Normal * p->m_Velocity < 0)) {
        Vec2f f_normal = (m_Normal * p->m_Force) * m_Normal;
        Vec2f f_tangent = p->m_Force - f_normal;
        p->m_Force = f_tangent - r * f_normal;
    }
}

void CollisionLine::draw()
{
    Vec2f vector = Vec2f(-m_Normal[1], m_Normal[0]);
    vector /= vector*vector;
	int N = 64;
	glBegin( GL_LINES );
	glColor3f(0.6, 0.7, 0.8);
	glVertex2f(m_Position[0] - vector[0]*N, m_Position[1] - vector[1]*N);
	glColor3f(0.6, 0.7, 0.8);
	glVertex2f(m_Position[0] + vector[0]*N, m_Position[1] + vector[1]*N);
	glEnd();
}
