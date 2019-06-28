#ifndef RAY_H
#define RAY_H
#include"Geometry.h"
#include"BoundingSphere.h"
#include"AABB.h"
#include"OBB.h"

class Ray {
public:
	Ray() : p(0.f),d(0.f) {}
	Ray(const glm::vec3& p, const glm::vec3& d) : p(p), d(d) {};
	bool intersect(const BoundingSphere& sphere, float& t) const;
	bool intersect(const AABB& box, float& t) const;
	bool intersect(const OBB& box, float& t) const;
	inline const glm::vec3& getOrigin() const { return p; }
	inline const glm::vec3& getDirection() const { return d; }
private:
	glm::vec3 p;
	glm::vec3 d;
	bool intersect(const Plane& p, float& ans) const;
};
#endif //RAY_H