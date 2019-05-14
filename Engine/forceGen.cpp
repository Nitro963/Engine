#include "forceGen.h"
#include <iostream>

void ForceRegistry::add(RigidBody * body, ForceGenerator * fg){
	registrations.push_back(ForceRegistration(body, fg));
}

void ForceRegistry::remove(RigidBody * body, ForceGenerator * fg){
	registrations.remove(ForceRegistration(body, fg));
}

void ForceRegistry::clear(){
	registrations.clear();
}

void ForceRegistry::updateForces(float duration){
	for (auto reg : registrations)
		reg.fg->updateForce(reg.body ,duration);
}

void GravityForce::updateForce(RigidBody * body ,float duration){
	body->applyForce(gravity * body->getMass());
}

void DragForce::updateForce(RigidBody * RigidBody, float duration){
	glm::vec3 force = RigidBody->getVelocity();
	// Calculate the total drag coefficient.
	float dragCoeff = glm::length(force);
	dragCoeff = k1 * dragCoeff + k2 * dragCoeff * dragCoeff;
	// Calculate the final force and apply it.
	force = glm::normalize(force);
	force *= -dragCoeff;
	RigidBody->applyForce(force);
}

void Spring::updateForce(RigidBody * RigidBody, float duration){
	// Calculate the vector of the spring.
	glm::vec3 force = RigidBody->transformLocal(*connectionPoint);

	force -= other->transformLocal(*otherConnectionPoint);

	float magnitude = glm::length(force);
	magnitude =	glm::abs(magnitude - restLength);
	magnitude *= springConstant;
	// Calculate the final force and apply it.
	force = glm::normalize(force);
	force *= -magnitude;
	RigidBody->applyForce(*connectionPoint ,force);
	other->applyForce(*otherConnectionPoint, -force);
}

void AnchoredSpring::updateForce(RigidBody * RigidBody, float duration){
	// Calculate the vector of the spring.
	glm::vec3 force = RigidBody->transformLocal(*connectionPoint);
	force -= *anchor;

	// Calculate the magnitude of the force.
	float magnitude = glm::length(force);
	magnitude = glm::abs(magnitude - restLength);
	magnitude *= springConstant;
	
	// Calculate the final force and apply it.
	force = glm::normalize(force);
	force *= -magnitude;
	RigidBody->applyForce(*connectionPoint ,force);
}

void FakeSpring::updateForce(RigidBody * rigidBody, float duration){
	// Calculate the relative position of the particle to the anchor.
	glm::vec3 position = rigidBody->transformLocal(connectionPoint) - *initialPosition;
	position -= *anchor;
	// Calculate the constants and check whether they are in bounds.
	float gamma = 0.5f * glm::sqrt(4 * springConstant - damping*damping);
	if (gamma - EPSILON < EPSILON) return;
	glm::vec3 c = position * (damping / (2.0f * gamma)) +
		rigidBody->getVelocity() * (1.0f / gamma);
	// Calculate the target position.
	glm::vec3 target = position * glm::cos(gamma * duration) +
		c * glm::sin(gamma * duration);
	target *= glm::exp(-0.5f * duration * damping);
	// Calculate the resulting acceleration and therefore the force
	glm::vec3 accel = (target - position) * (1.0f / duration*duration) -
		rigidBody->getVelocity() * duration;
	rigidBody->applyForce(connectionPoint,accel * rigidBody->getMass());
}
