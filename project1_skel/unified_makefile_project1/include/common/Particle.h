#pragma once

#include <gfx/vec2.h>

class Particle
{
public:

	Particle(const Vec2f &ConstructPos, float Mass);
	virtual ~Particle(void);

	void reset();
    void set_state(Vec2f pos, Vec2f vel);
    void draw();

    Vec2f m_ConstructPos;
	Vec2f m_Position;
	Vec2f m_Velocity;
	float m_Mass;
	Vec2f m_Force;
};
