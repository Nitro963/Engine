#ifndef AABB_INCLUDE
#define AABB_INCLUDE
#include"glm\glm.hpp"
#include"IntersectData.h"

class AABB {
public:
	AABB(const glm::vec3& minExtents ,const glm::vec3& maxExtents):m_minExtents(minExtents) ,m_maxExtents(maxExtents){}
	inline const glm::vec3& getMinExtents() const { return m_minExtents; }
	inline const glm::vec3& getMaxExtents() const { return m_maxExtents; }
	intersectData intersectAABB(const AABB& other);
private:
	glm::vec3 m_minExtents;
	glm::vec3 m_maxExtents;
};
#endif // !AABB_INCLUDE

