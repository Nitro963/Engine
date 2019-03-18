#ifndef BOUNDING_SPHERE
#define BOUNDING_SPHERE
#include "glm\glm.hpp"
#include "IntersectData.h"

class boundingSphere {
public:
	inline boundingSphere(const glm::vec3& center, float radius) : m_center(center), m_radius(radius) {};
	inline float getRadius() const { return m_radius; }
	inline const glm::vec3 getCenter()const { return m_center; }
	intersectData intersectBoundingSphere(boundingSphere& other);
private:
	glm::vec3 m_center;
	float m_radius;
};
#endif // !
