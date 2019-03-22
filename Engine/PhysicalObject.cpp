#include "PhysicalObject.h"
#include <iostream>
void physicalObject::integrate(){
	m_velocity += m_acceleration;
	m_position += m_velocity;
}

void physicalObject::applyForce(const glm::vec3 & force){
	m_acceleration += force / m_mass;
}
