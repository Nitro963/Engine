#include "BoundingSphere.h"

bool BoundingSphere::intersect(const BoundingSphere & b) const{
	// Calculate squared distance between centers
	glm::vec3 d = c - b.c;
	float dist2 = glm::dot(d, d);
	// Spheres intersect if squared distance is less than squared sum of radii
	float radiusSum = r + b.r;
	return dist2 - radiusSum * radiusSum < EPSILON;
}

bool BoundingSphere::intersect(const OBB &b) const{

	// Find point p on OBB closest to sphere center
	point p = b.closestPoint(c);

	// Sphere and OBB intersect if the (squared) distance from sphere
	// center to point p is less than the (squared) sphere radius
	glm::vec3 v = p - c;
	return dot(v, v) - r * r < EPSILON;
}

CollisionManifold BoundingSphere::findCollisionFeatures(const BoundingSphere & b) const{
	CollisionManifold res;
	res.colliding = intersect(b);
	if (!res.colliding)
		return res;
	glm::vec3 d = b.c - c;
	res.depth = r + b.r - glm::length(d);
	res.normal = glm::normalize(d);
	res.contacts.push_back(c + res.normal * (r - res.depth));
	return res;
}

CollisionManifold BoundingSphere::findCollisionFeatures(const OBB & b) const{
	CollisionManifold res;
	res.colliding = intersect(b);
	if (!res.colliding)
		return res;
	// Find point p on OBB closest to sphere center
	point p = b.closestPoint(c);

	res.contacts.push_back(p);

	res.normal = glm::normalize(p - c);
	point outsidePoint = c + res.normal * r;
	res.depth = glm::length(p - outsidePoint);
	return res;
}

point BoundingSphere::closestPoint(const point & p){
	glm::vec3 n = p - c;
	if (glm::dot(n, n) - r * r < EPSILON)
		return p;
	return c + normalize(n) * r;
}

void BoundingSphere::sync(const point & c){
	this->c = c;
}

void BoundingSphere::update(const point & c, float r){
	this->c = c;
	this->r = r;
}
