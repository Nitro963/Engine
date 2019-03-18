#ifndef PLANE_H
#define PLANE_H
#include"glm\glm.hpp"
#include"IntersectData.h"
#include "BoundingSphere.h"

class plane {
public:
	plane(glm::vec3 normal, float distance) :m_normal(normal), m_distance(distance) {}
	plane normalized() const;
	intersectData intersectSphere(const boundingSphere sphere);
	inline const glm::vec3& getNormal() const { return m_normal; }
	inline float getDistance() const { return m_distance; }
private:
	glm::vec3 m_normal;
	float m_distance;

};
#endif // !PLANE_H
