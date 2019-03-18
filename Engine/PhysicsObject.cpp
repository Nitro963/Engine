#include "PhysicsObject.h"

void physicsObject::integrate(float delta){
	m_position += m_velocity * delta;
}
