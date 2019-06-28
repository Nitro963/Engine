#include "Joint.h"

void JointRegistry::add(RigidBody * rigidBody, Joint * joint){
	registrations.push_back(JointRegistration(rigidBody, joint));
}

void JointRegistry::remove_if(bool(*fun)(RigidBody*&)) {
	for (auto it = registrations.begin(); it != registrations.end();)
		if (fun(it->body)) {
			auto tmp = it;
			++it;
			registrations.erase(tmp);
		}
		else
			++it;
}

void JointRegistry::remove_if(bool(*fun)(JointRegistration &R)) {
	for (auto it = registrations.begin(); it != registrations.end();)
		if (fun(*it)) {
			auto tmp = it;
			++it;
			registrations.erase(tmp);
		}
		else
			++it;
}

void JointRegistry::clear(){
	registrations.clear();
}

void JointRegistry::solveConstraints(float duration){
	for (auto& reg : registrations)
		reg.joint->solveConstraint(reg.body);
}

void FixedJoint::solveConstraint(RigidBody * body){
	body->setPosition(pos);
	body->setVelocity(glm::vec3(0.f));
}
