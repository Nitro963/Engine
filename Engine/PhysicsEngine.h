#ifndef PHYSICSENGINE_H
#define PHYSICSENGINE_H
#include"glm\glm.hpp"
#include"PhysicalObject.h"
#include"Plane.h"
#include<vector>

class physicsEngine {
public:
	physicsEngine(float scale) { 
		edges.push_back(plane(glm::vec3(0, 1, 0), 1.f / scale));
		edges.push_back(plane(glm::vec3(0, -1, 0), 1.f / scale));
		edges.push_back(plane(glm::vec3(1, 0, 0), 1.f / scale));
		edges.push_back(plane(glm::vec3(-1, 0, 0), 1.f / scale));
		this->scale = scale; 
	}
	void addObject(const physicalObject& obj);
	void simulate(float frameRate);
	inline const std::vector<physicalObject>& const getObjects() { return objects; }
private:
	void applyGravitationalAttraction();
	void applyNormalForce();
	//void applyDragForce();
	std::vector<physicalObject> objects;
	const double G = -6.67428 * 1e-11;
	double scale = 1;
	std::vector<plane> edges;
};
#endif // !PHYSICSENGINE_H
