#include "BoundingSphere.h"

intersectData boundingSphere::intersectBoundingSphere(boundingSphere& other){
	float radiusDistance = m_radius + other.getRadius();
	float centerDistance = glm::length(m_center - other.getCenter());
	return intersectData(centerDistance <= radiusDistance ,centerDistance - radiusDistance);
}
