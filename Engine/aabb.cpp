#include "aabb.h"
#include <algorithm>

intersectData AABB::intersectAABB(const AABB & other){
	glm::vec3 distance1 = other.getMinExtents() - m_maxExtents;
	glm::vec3 distance2 = m_minExtents - other.getMaxExtents();
	glm::vec3 distance = glm::max(distance1, distance2);
	float maxDistance = std::max(distance.x, std::max(distance.y, distance.z));
	return intersectData(maxDistance < 0 ,maxDistance);
}
