#include "PhysicsEngine.h"

void physicsEngine::addObject(const physicsObject& obj){
	objects.push_back(obj);
}

void physicsEngine::simulate(float delta){
	for (auto& x : objects)
		x.integrate(delta);
}
