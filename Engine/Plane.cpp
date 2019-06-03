#include "Plane.h"

bool plane::intersect(const boundingSphere & s) const {
	// For a normalized plane (|p.n| = 1), evaluating the plane equation
	// for a point gives the signed distance of the point to the plane
	// If sphere center within +/-radius from plane, plane intersects sphere
	return abs(distPoint(s.getCenter())) - s.getRadius() <= EPSILON;
}

bool plane::insideNegHalfSpace(const boundingSphere & s) const {
	return distPoint(s.getCenter()) < -s.getRadius();
}

bool plane::intersectNegHalfSpace(const boundingSphere & s) const {
	return distPoint(s.getCenter()) - s.getRadius() < EPSILON;
}

bool plane::intersect(const OBB& b) const{
	// Compute the projection interval radius of b onto L(t) = b.c + t * p.n
	float r = b.getHalfExtents().x * glm::abs(glm::dot(n, b.getLocalX())) +
		b.getHalfExtents().y * glm::abs(glm::dot(n, b.getLocalY())) +
		b.getHalfExtents().z * glm::abs(glm::dot(n, b.getLocalZ()));

	// Compute distance of box center from the plane
	// Intersection occurs when distance s falls within [-r,+r] interval
	return glm::abs(distPoint(b.getCenter())) - r < EPSILON;
}

bool plane::intersect(const AABB & b) const{
	// Compute the projection interval radius of b onto L(t) = b.c + t * p.n
	float r = b.getXRadius() * glm::abs(n[0]) + b.getYRadius() * glm::abs(n[1]) + b.getZRadius() * glm::abs(n[2]);
	// Compute distance of box center from plane
	// Intersection occurs when distance s falls within [-r,+r] interval
	return glm::abs(distPoint(b.getCenter())) - r < EPSILON;
}

bool plane::intersect(const Line & seg) const{
	// Compute the t value for the directed line ab intersecting the plane
	float t = (d - glm::dot(n ,seg.a)) / (glm::dot(seg.b - seg.a, n) + EPSILON);
	// If t in [0..1] then the plane intersect line segment
	if (t >= 0.f && t <= 1.f)
		return true;
	return false;
}

bool plane::clip(const Line & seg, point & p) const{
	glm::vec3 ab = seg.b - seg.a;
	// Compute the t value for the directed line ab intersecting the plane
	float t = (d - glm::dot(n, seg.a)) / (glm::dot(ab, n) + EPSILON);
	// If t in [0..1] compute and return intersection point
	if (t >= 0.f && t <= 1.f) {
		p = seg.a + t * ab;
		return true;
	}
	return false;
}

point plane::closestPoint(const point& p) const {
	return p - distPoint(p) * n;
}

float plane::distPoint(const point& q) const {
	return glm::dot(n, q) - d;
}
