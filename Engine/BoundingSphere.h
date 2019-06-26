#ifndef BOUNDINGSPHERE_H
#define BOUNDINGSPHERE_H
#include "Geometry.h"
#include "AABB.h"
#include "OBB.h"

class BoundingSphere {
public:
	//Given Sphere center and radius
	BoundingSphere(const point& c, float r):c(c), r(r){}

	//generate contacts
	CollisionManifold findCollisionFeatures(const BoundingSphere& b) const;

	CollisionManifold findCollisionFeatures(const OBB& b) const;

	// Given point p, return the point q on or in the Sphere that is closest to p
	point closestPoint(const point& p);

	void sync(const point& c);
	void update(const point& c, float r);

private:
	// Determine whether the sphere intersects sphere b
	bool intersect(const BoundingSphere& b) const;

	// Determine whether the sphere intersects OBB b
	bool intersect(const OBB& b) const;
	
	point c;// Sphere center
	float r;// Spherer radius

	friend class OBB;
	friend class Plane;
	friend class AABB;
	friend class Ray;
};
#endif // !BOUNDINGSPHERE_H
