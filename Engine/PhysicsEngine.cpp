#include "PhysicsEngine.h"
#include <iostream>

void physicsEngine::addObject(const physicalObject& obj){
	objects.push_back(obj);
}

void physicsEngine::simulate(float frameRate) {
	for(int i = 0 ; i < objects.size(); i++)
		for (int j = i + 1; j < objects.size(); j++) {
			glm::vec3 force = objects[i].getPosition() - objects[j].getPosition();
			double distance = glm::length(force);
			force = glm::normalize(force);
			force *= G * objects[j].getMass() *objects[i].getMass() / (distance * distance);
			force *= scale / (frameRate * frameRate);
			objects[i].applyForce(force);
			force *= -1;
			objects[j].applyForce(force);
		}
	for (int i = 0; i < objects.size() - 1; i++)
			objects[i].integrate();

}

