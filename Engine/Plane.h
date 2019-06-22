#ifndef PLANE_H
#define PLANE_H
#include "Geometry.h"
#include "BoundingSphere.h"
#include "AABB.h"
#include "OBB.h"
#include <iostream>
class Plane {
public:
	//default constructor
	Plane() : n(), d(0) {}
	// Given three noncollinear points (ordered ccw), compute plane equation
	Plane(const point& a, const point& b, const point& c) {
		n = glm::normalize(glm::cross(b - a, c - a));
		d = glm::dot(n, a);
	}
	
	//Given a point on the plane and a normal
	Plane(const glm::vec3& n, const point& a) {
		this->n = n;
		d = glm::dot(this->n, a);
	}
	
	//Given a normal and the distance
	Plane(const glm::vec3& n, float d) :n(n), d(d) {}

	//Points x on the plane satisfy Dot(n, x) = d
	inline bool inside(const point& p) const { return glm::abs(glm::dot(n, p) - d) < EPSILON; }
	
	// Determine whether the plane intersects sphere s
	bool intersect(const BoundingSphere& s) const;
	
	// Determine whether sphere s is fully behind (inside negative halfspace of) the plane
	bool insideNegHalfSpace(const BoundingSphere& s) const;
	
	// Determine whether sphere s intersects negative halfspace of the plane 
	bool intersectNegHalfSpace(const BoundingSphere& s) const;
	
	// Test if OBB b intersects the plane
	bool intersect(const OBB& b) const;

	// Test if AABB b intersects the plane
	bool intersect(const AABB& b) const;

	// Test if Line segement intersect the plane
	bool intersect(const Line& seg) const;

	// if Line segement intersect the plane return p the intersection point
	bool clip(const Line& seg, point& p) const;

	// Given point p, return the point q on the plane that is closest to p
	point closestPoint(const point& p) const;

	// calculate the distance between the plane and a point q
	float distPoint(const point& q) const;

private:
	glm::vec3 n; // Plane normal. Points x on the plane satisfy Dot(n,x) = d
	float d; // d = dot(n,p) for a given point p on the plane
};

#endif // !PLANE_H
