#include "BoundingSphere.h"

bool boundingSphere::testSphere(const boundingSphere & b ,CollisionData& info) const{
	// Calculate squared distance between centers
	glm::vec3 d = c - b.getCenter();
	float dist2 = glm::dot(d, d);
	// Spheres intersect if squared distance is less than squared sum of radii
	float radiusSum = r + b.r;
	if (dist2 - radiusSum * radiusSum < EPSILON) {
		info.contactNormal = glm::normalize(d);
		info.collisionPoint = b.c + d * 0.5f;
		return true;
	}
	return false;
}

bool boundingSphere::testAABB(const AABB & b) const{

	// Find point p on AABB closest to sphere center
	point p = b.closestPoint(c);

	// Sphere and AABB intersect if the (squared) distance from sphere
	// center to point p is less than the (squared) sphere radius
	glm::vec3 v = p - c;
	
	return glm::dot(v ,v) - r * r < EPSILON;
}

bool boundingSphere::testOBB(const OBB & b , CollisionData& info) const{

	// Find point p on OBB closest to sphere center
	point p = b.closestPoint(c);

	// Sphere and OBB intersect if the (squared) distance from sphere
	// center to point p is less than the (squared) sphere radius
	glm::vec3 v = p - c;
	if (dot(v, v) - r * r < EPSILON) {
		info.collisionPoint = p;
		info.contactNormal = glm::normalize(p - c);
		info.penetrationDepth = 0;
		return true;
	}
	return false;
}

point boundingSphere::closestPoint(const point & p){
	glm::vec3 n = p - c;
	if (glm::dot(n, n) - r * r < EPSILON)
		return p;
	return n * 0.5f + c;
}
