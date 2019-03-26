#include "PhysicalObject.h"
#include <iostream>

void physicalObject::integrate(float frameRate){
	m_velocity += m_acceleration / (frameRate * frameRate);
	m_position += m_velocity;
	m_acceleration *= 0;
}

void physicalObject::applyForce(const glm::vec3 & force){
	m_acceleration += force / m_mass;
}

void physicalObject::reverseVelocity(){
	m_velocity *= -1;
	m_position.y = -9;
}

