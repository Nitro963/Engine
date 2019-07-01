#include "forceGen.h"
#include <iostream>

const float GravityForce::G = 6.67408 * 1e-11;

void ForceRegistry::add(RigidBody * body, ForceGenerator * fg) {
	registrations.push_back(ForceRegistration(body, fg));
}

void ForceRegistry::remove_if(bool(*fun)(RigidBody*&)) {
	for (auto it = registrations.begin(); it != registrations.end();)
		if (fun(it->body)) {
			auto tmp = it;
			++it;
			registrations.erase(tmp);
		}
		else
			++it;
}

void ForceRegistry::remove_if(bool(*fun)(ForceRegistration &R)) {
	for (auto it = registrations.begin(); it != registrations.end();)
		if (fun(*it)) {
			auto tmp = it;
			++it;
			registrations.erase(tmp);
		}
		else
			++it;
}

void ForceRegistry::remove(RigidBody *& body){
	for(auto it = registrations.begin();it != registrations.end();)
		if (it->body == body) {
			auto tmp = it;
			++it;
			registrations.erase(tmp);
		}
		else
			++it;
}

void ForceRegistry::remove(ForceGenerator * fg){
	for (auto it = registrations.begin(); it != registrations.end();)
		if (it->fg == fg) {
			auto tmp = it;
			++it;
			registrations.erase(tmp);
		}
		else
			++it;
}

void ForceRegistry::clear() {
	registrations.clear();
}

void ForceRegistry::updateForces(float duration) {
	for (auto& reg : registrations)
		reg.fg->updateForce(reg.body, duration);
}

void GravityForce::updateForce(RigidBody * body, float duration) {
	body->applyImpulse(gravity * body->getMass() * duration);
}

void DragForce::updateForce(RigidBody * RigidBody, float duration) {
	glm::vec3 force = RigidBody->getVelocity();
	// Calculate the total drag coefficient.
	float dragCoeff = -C * glm::dot(force, force);
	// Calculate the final force and apply it.
	force = dragCoeff * glm::normalize(force);
	RigidBody->applyImpulse(force * duration);
}

void MotorJoint::updateForce(RigidBody * RigidBody, float duration) {
	if(RigidBody->contains(pt))
		RigidBody->applyImpulse(force * duration,glm::cross(pt, force) * duration);
}

void TimedMotorJoint::updateForce(RigidBody * RigidBody, float duration){
	if (!RigidBody->contains(pt))
		return;
	if (t > EPSILON) {
		RigidBody->applyImpulse(force * duration, glm::cross(pt, force) * duration);
		t -= duration;
	}
}