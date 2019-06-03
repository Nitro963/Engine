#ifndef BOUNDINGSPHERE_H
#define BOUNDINGSPHERE_H
#include "Geometry.h"
#include "AABB.h"
#include "OBB.h"

class boundingSphere {
public:
	boundingSphere(const point& c ,float r):c(c) ,r(r){}
	inline const point& getCenter() const { return c; }
	inline const float getRadius() const { return r; }

	//generate contacts
	CollisionManifold findCollisionFeatures(const boundingSphere& b) const;

	CollisionManifold findCollisionFeatures(const OBB& b) const;

	// Given point p, return the point q on or in the Sphere that is closest to p
	point closestPoint(const point& p);

	void sync(const point& c);
	void update(const point& c, float r);

	friend class OBB;
private:
	// Determine whether the sphere intersects sphere b
	bool intersect(const boundingSphere& b) const;
	// Determine whether the sphere intersects AABB b
	bool intersect(const AABB& b) const;
	// Determine whether the sphere intersects OBB b
	bool intersect(const OBB& b) const;

	point c;
	float r;
};
#endif // !BOUNDINGSPHERE_H
