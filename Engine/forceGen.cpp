#include "forceGen.h"
#include <iostream>

const float GravityForce::G = 6.67408 * 1e-11;

void ForceRegistry::add(RigidBody * body, ForceGenerator * fg){
	registrations.push_back(ForceRegistration(body, fg));
}

void ForceRegistry::remove_if(bool (*fun)(RigidBody*&)){
	for (auto it = registrations.begin(); it != registrations.end(); ++it) {
		if (fun(it->body)) {
			auto tmp = it;
			++it;
			registrations.erase(tmp);
		}
	}
}

void ForceRegistry::remove_if(bool(*fun)(ForceRegistration &R)){
	for (auto it = registrations.begin(); it != registrations.end(); ++it) {
		if (fun(*it)) {
			auto tmp = it;
			++it;
			registrations.erase(tmp);
		}
	}
}

void ForceRegistry::clear(){
	registrations.clear();
}

void ForceRegistry::updateForces(){
	for (auto& reg : registrations)
		reg.fg->updateForce(reg.body);
}

void GravityForce::updateForce(RigidBody * body){
	body->applyForce(gravity * body->getMass());
}

void DragForce::updateForce(RigidBody * RigidBody){
	glm::vec3 force = RigidBody->getVelocity();
	// Calculate the total drag coefficient.
	float dragCoeff = - C * glm::dot(force ,force);
	// Calculate the final force and apply it.
	force = dragCoeff * glm::normalize(force);
	RigidBody->applyForce(force);
}

void MotorJoint::updateForce(RigidBody * RigidBody){
	RigidBody->applyForce(pt, force);
}
