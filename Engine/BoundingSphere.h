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
	// Determine whether the sphere intersects sphere 
	bool testSphere(const boundingSphere& b ,CollisionData& ret) const;
	bool testAABB(const AABB& b) const;
	bool testOBB(const OBB& b ,CollisionData& info) const;

	point closestPoint(const point& p);
private:
	point c;
	float r;
};
#endif // !BOUNDINGSPHERE_H
