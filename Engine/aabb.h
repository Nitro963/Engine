#pragma once
#ifndef AABB_H
#define AABB_H
#include"Geometry.h"

class AABB {
public:
	AABB(const point& center, const glm::vec3& radius) :c(center) ,r(radius) {}
	inline const point& getCenter() const { return c; }
	inline const float getXRadius() const { return r[0]; }
	inline const float getYRadius() const { return r[1]; }
	inline const float getZRadius() const { return r[2]; }
	bool intersect(const AABB& b) const;
	AABB updateAABB(const glm::mat3x3& rotation, const glm::vec3& translation) const;
	point closestPoint(const point& p) const;

private:
	point c;
	glm::vec3 r;

};

#endif // !AABB_H
