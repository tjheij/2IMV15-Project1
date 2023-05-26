#pragma once

#include <gfx/vec2.h>
#include "Particle.h"

class CollisionLine
{
public:

	CollisionLine(const Vec2f position, const Vec2f normal, const float e);
	
    void collisionWith(Particle* p);
    void collisionForceWith(Particle* p);

    void draw();

	const Vec2f m_Position;
	const Vec2f m_Normal;
    const float m_E;
};
