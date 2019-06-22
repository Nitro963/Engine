#include "Plane.h"

bool Plane::intersect(const BoundingSphere & s) const {
	// For a normalized plane (|p.n| = 1), evaluating the plane equation
	// for a point gives the signed distance of the point to the plane
	// If sphere center within +/-radius from plane, plane intersects sphere
	return abs(distPoint(s.c)) - s.r < EPSILON;
}

bool Plane::insideNegHalfSpace(const BoundingSphere & s) const {
	return distPoint(s.c) < -s.r;
}

bool Plane::intersectNegHalfSpace(const BoundingSphere & s) const {
	return distPoint(s.c) - s.r < EPSILON;
}

bool Plane::intersect(const OBB& b) const{
	// Compute the projection interval radius of b onto L(t) = b.c + t * p.n
	float r = b.halfExtents.x * glm::abs(glm::dot(n, b.u[0])) +
		b.halfExtents.y * glm::abs(glm::dot(n, b.u[1])) +
		b.halfExtents.z * glm::abs(glm::dot(n, b.u[2]));

	// Compute distance of box center from the plane
	// Intersection occurs when distance s falls within [-r,+r] interval
	return glm::abs(distPoint(b.c)) - r < EPSILON;
}

bool Plane::intersect(const AABB & b) const{
	// Compute the projection interval radius of b onto L(t) = b.c + t * p.n
	float r = b.halfExtents.x * glm::abs(n[0]) + b.halfExtents.x * glm::abs(n[1]) + b.halfExtents.z * glm::abs(n[2]);
	// Compute distance of box center from plane
	// Intersection occurs when distance s falls within [-r,+r] interval
	return glm::abs(distPoint(b.center)) - r < EPSILON;
}

bool Plane::intersect(const Line & seg) const{
	// Compute the t value for the directed line intersecting the plane
	glm::vec3 ab = seg.b - seg.a;
	float t = (d - glm::dot(n, seg.a)) / glm::dot(n, ab);
	// If t in [0..1] compute and return intersection point
	if (t >= 0.0f && t <= 1.0f) {
		return true;
	}
	// Else no intersection
	return false;
}

bool Plane::clip(const Line & seg, point & p) const{
	// Compute the t value for the directed line intersecting the plane
	glm::vec3 ab = seg.b - seg.a;

	float t = (d - glm::dot(n, seg.a)) / glm::dot(n, ab);
	// If t in [0..1] compute and return intersection point
	if (t >= 0.0f && t <= 1.0f) {
		p = seg.a + ab * t;
		return true;
	}
	// Else no intersection
	return false;
}

point Plane::closestPoint(const point& p) const {
	return p - distPoint(p) * n;
}

float Plane::distPoint(const point& q) const {
	return glm::dot(n, q) - d;
}
