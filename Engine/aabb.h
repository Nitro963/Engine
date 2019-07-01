#pragma once
#ifndef AABB_H
#define AABB_H
#include"Geometry.h"
class OBB;
class BoundingSphere;
class AABB {
public:
	AABB() : c(), halfExtents(){}
	AABB(const point& center, glm::vec3 halfExtents) :c(center), halfExtents(halfExtents), mn(c - halfExtents), mx(c + halfExtents){}
	AABB(const glm::vec4& minV, const glm::vec4& maxV) :mn(minV), mx(maxV), c((minV + maxV) * 0.5f), halfExtents((maxV - minV) * 0.5f) {}
	inline const glm::vec3& getMin() const { return mn; }
	inline const glm::vec3& getMax() const { return mx; }
	inline glm::vec3 getExtents() const { return halfExtents * 2.f; }
	inline const glm::vec3& getHalfExtents() const { return halfExtents; }
	inline const point& getCenter() const { return c; }
	bool intersect(const AABB& b) const;
	bool contains(const point& p) const;
	bool contains(const OBB& b) const;
	bool contains(const BoundingSphere& s) const;

	friend class Plane;
	friend class OBB;
	friend class BoundingSphere;
	friend class Ray;
private:
	point c;
	glm::vec3 halfExtents;
	glm::vec3 mn;
	glm::vec3 mx;
};

#endif // !AABB_H
