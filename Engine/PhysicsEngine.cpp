#include "PhysicsEngine.h"
#include <iostream>

void physicsEngine::addObject(const physicalObject& obj){
	objects.push_back(obj);
}

void physicsEngine::simulate(float frameRate) {
	for (int i = 0; i < objects.size(); i++)
		objects[i].integrate(frameRate);
}

void physicsEngine::applyGravitationalAttraction(){
	for (int i = 0; i < objects.size(); i++)
		for (int j = i + 1; j < objects.size(); j++) {
			glm::vec3 force = objects[i].getPosition() - objects[j].getPosition();
			double distance = glm::length(force);
			force = glm::normalize(force);
			force *= G * objects[j].getMass() *objects[i].getMass() / (distance * distance);
			objects[i].applyForce(force);
			force *= -1;
			objects[j].applyForce(force);
		}
}

void physicsEngine::applyNormalForce(){
	for(auto edge : edges)
		for(int i = 0 ; i < objects.size() ; i++)
			if (edge.intersectSphere(boundingSphere(objects[i].getPosition(), 1)).getDoseIntersect()) {
				glm::vec3 normalForce = objects[i].getAcceleration() * objects[i].getMass() * -1.f;
				objects[i].applyForce(normalForce);
				objects[i].reverseVelocity();
		}
}
