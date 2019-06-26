#ifndef RAY_H
#define RAY_H
#include"Geometry.h"
#include"BoundingSphere.h"
#include"AABB.h"
#include"OBB.h"

class Ray {
private:
	glm::vec3 p;
	glm::vec3 d;
	bool intersect(const Plane& p, float& ans) const;
public:
	Ray() {}
	Ray(const glm::vec3& p, const glm::vec3& d) : p(p), d(d) {};
	bool intersect(const BoundingSphere& sphere, glm::vec3& ret) const;
	bool intersect(const AABB& box, glm::vec3& ret) const;
	bool intersect(const OBB& box, glm::vec3& ret) const;
	inline const glm::vec3& getOrigin() const { return p; }
	inline const glm::vec3& getDirection() const { return d; }
};
#endif //RAY_H