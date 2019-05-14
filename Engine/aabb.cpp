#include "AABB.h"

bool AABB::testAABB(const AABB & b) const{
	
	if (glm::abs(c.x - b.getCenter().x) > getXRadius() + b.getXRadius()) return false;
	if (glm::abs(c.y - b.getCenter().y) > getYRadius() + b.getYRadius()) return false;
	if (glm::abs(c.z - b.getCenter().z) > getZRadius() + b.getZRadius()) return false;

	return true;
}

// Transform AABB by the matrix m and translation t,
// find maximum extents, and store result into a new AABB.
AABB AABB::updateAABB(const glm::mat3x3& rotation,const glm::vec3& translation) const{
	point c;
	glm::vec3 r;
	for (int i = 0; i < 3; i++) {
		c[i] = translation[i];
		r[i] = 0.0f;
		for (int j = 0; j < 3; j++) {
			c[i] += rotation[i][j] * this->c[j];
			r[i] += glm::abs(rotation[i][j]) * this->r[j];
		}
	}
	return AABB(c, r);
}

// Given point p, return the point q on or in the AABB that is closest to p
point AABB::closestPoint(const point& p) const{
	// For each coordinate axis, if the point coordinate value is
	// outside box, clamp it to the box, else keep it as is
	return glm::clamp(p ,c - r ,c + r);
}
