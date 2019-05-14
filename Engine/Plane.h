#ifndef PLANE_H
#define PLANE_H
#include "Geometry.h"
#include "BoundingSphere.h"
#include "AABB.h"
#include "OBB.h"
#include <iostream>
class plane {
public:
	// Given three noncollinear points (ordered ccw), compute plane equation
	plane(const point& a, const point& b, const point& c) {
		n = glm::normalize(glm::cross(b - a, c - a));
		d = glm::dot(n, a);
	}

	//Points x on the plane satisfy Dot(n, x) = d
	inline bool inside(const point& p) { return glm::abs(glm::dot(n, p) - d) < EPSILON; }
	
	// Determine whether the plane intersects sphere s
	bool testSphere(const boundingSphere& s) const;
	
	// Determine whether sphere s is fully behind (inside negative halfspace of) the plane
	bool insideNegHalfSpace(const boundingSphere& s) const;
	
	// Determine whether sphere s intersects negative halfspace of the plane 
	bool intersectNegHalfSpace(const boundingSphere& s) const;
	
	// Test if OBB b intersects the plane
	bool testOBB(const OBB& b) const;

	// Test if AABB b intersects plane p
	bool testAABB(const AABB& b) const;

	// Given point p, return the point q on the plane that is closest to p
	point closestPoint(const point& p) const;

	// calculate the distance between the plane and a point q
	float distPoint(const point& q) const;

private:
	glm::vec3 n; // Plane normal. Points x on the plane satisfy Dot(n,x) = d
	float d; // d = dot(n,p) for a given point p on the plane
};

#endif // !PLANE_H
