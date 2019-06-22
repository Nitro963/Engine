#pragma once
#ifndef AABB_H
#define AABB_H
#include"Geometry.h"
class OBB;
class BoundingSphere;
class AABB {
public:
	AABB() : center(), halfExtents(){}
	AABB(const point& center, glm::vec3 halfExtents) :center(center), halfExtents(halfExtents) {}
	AABB(const glm::vec4& minV, const glm::vec4& maxV) {
		center = (minV + maxV) * 0.5f;
		halfExtents = (maxV - minV) * 0.5f;
	}
	inline glm::vec3 getMin() const { return glm::vec3(center - halfExtents); }
	inline glm::vec3 getMax() const { return glm::vec3(center + halfExtents); }
	inline glm::vec3 getExtents() const { return halfExtents * 2.f; }
	inline const glm::vec3& getHalfExtents() const { return halfExtents; }
	inline const point& getCenter() const { return center; }
	bool intersect(const AABB& b) const;
	bool contains(const point& p) const;
	bool contains(const OBB& b) const;
	bool contains(const BoundingSphere& s) const;

	friend class Plane;
	friend class OBB;
	friend class BoundingSphere;
private:
	point center;
	glm::vec3 halfExtents;
};

#endif // !AABB_H
