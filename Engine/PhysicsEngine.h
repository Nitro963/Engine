#ifndef PHYSICSENGINE_H
#define PHYSICSENGINE_H
#include"glm\glm.hpp"
#include"PhysicsObject.h"
#include<vector>

class physicsEngine {
public:
	void addObject(const physicsObject& obj);
	void simulate(float delta);

private:
	std::vector<physicsObject> objects;
};
#endif // !PHYSICSENGINE_H
