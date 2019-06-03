#include "BoundingSphere.h"

bool boundingSphere::intersect(const boundingSphere & b) const{
	// Calculate squared distance between centers
	glm::vec3 d = c - b.getCenter();
	float dist2 = glm::dot(d, d);
	// Spheres intersect if squared distance is less than squared sum of radii
	float radiusSum = r + b.r;
	return dist2 - radiusSum * radiusSum < EPSILON;
}

bool boundingSphere::intersect(const AABB & b) const{

	// Find point p on AABB closest to sphere center
	point p = b.closestPoint(c);

	// Sphere and AABB intersect if the (squared) distance from sphere
	// center to point p is less than the (squared) sphere radius
	glm::vec3 v = p - c;
	
	return glm::dot(v ,v) - r * r < EPSILON;
}

bool boundingSphere::intersect(const OBB &b) const{

	// Find point p on OBB closest to sphere center
	point p = b.closestPoint(c);

	// Sphere and OBB intersect if the (squared) distance from sphere
	// center to point p is less than the (squared) sphere radius
	glm::vec3 v = p - c;
	return dot(v, v) - r * r < EPSILON;
}

CollisionManifold boundingSphere::findCollisionFeatures(const boundingSphere & b) const{
	CollisionManifold res;
	res.colliding = intersect(b);
	if (!res.colliding)
		return res;
	glm::vec3 d = b.getCenter() - c;
	res.depth = glm::abs(glm::length(d) - r) * 0.5f;
	float dtp = r - res.depth;
	res.normal = glm::normalize(d);
	res.contacts.push_back(c + d * dtp);
	return res;
}

CollisionManifold boundingSphere::findCollisionFeatures(const OBB & b) const{
	CollisionManifold res;
	res.colliding = intersect(b);
	if (!res.colliding)
		return res;
	// Find point p on OBB closest to sphere center
	point p = b.closestPoint(c);
	res.contacts.push_back(p);
	point x = c;
	float magSQ = glm::dot(p - c ,p - c);
	//if mag ~ 0 then the closest point is at the center of the sphere
	if (magSQ < EPSILON2)
		x = p, p = b.getCenter();
	res.normal = glm::normalize(p - x);
	point outsidePoint = c - res.normal * r;
	res.depth = glm::length(p - outsidePoint) * 0.5;
	return res;
}

point boundingSphere::closestPoint(const point & p){
	glm::vec3 n = p - c;
	if (glm::dot(n, n) - r * r < EPSILON)
		return p;
	return n * 0.5f + c;
}

void boundingSphere::sync(const point & c){
	this->c = c;
}

void boundingSphere::update(const point & c, float r){
	this->c = c;
	this->r = r;
}
