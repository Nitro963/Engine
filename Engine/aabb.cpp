#include "AABB.h"
#include "OBB.h"
#include "BoundingSphere.h"

bool AABB::intersect(const AABB & b) const{
	
	if (glm::abs(center.x - b.center.x) > halfExtents.x + b.halfExtents.x) return false;
	if (glm::abs(center.y - b.center.y) > halfExtents.y + b.halfExtents.y) return false;
	if (glm::abs(center.z - b.center.z) > halfExtents.z + b.halfExtents.z) return false;

	return true;
}

bool AABB::contains(const point & p) const{
	glm::vec3 mn = getMin();
	glm::vec3 mx = getMax();
	return (p.x > mn.x && p.x < mx.x &&
		p.y > mn.y && p.y < mx.y &&
		p.z > mn.z && p.z < mx.z);
}

bool AABB::contains(const OBB& b) const{
	std::vector<point> vertices = b.getVertices();
	for (const auto& v : vertices) {
		if (!contains(v))
			return false;
	}
	return true;
}

bool AABB::contains(const BoundingSphere & s) const{
	glm::vec3 axis[]{
		glm::vec3(1 ,0 ,0) ,glm::vec3(0 ,1 ,0) ,glm::vec3(0 ,0 ,1)
	};
	for (int i = 0; i < 3; i++) {
		point p = s.c + s.r * axis[i];
		point q = s.c - s.r * axis[i];
		if (!contains(p))
			return false;
		if (!contains(q))
			return false;
	}
	return true;
}

