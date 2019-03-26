#include "Plane.h"

plane plane::normalized() const{
	float length = glm::length(m_normal);
	return plane(m_normal/length ,m_distance/length);
}

intersectData plane::intersectSphere(const boundingSphere sphere){
	float distanceFormSphereCenter = fabs(glm::dot(m_normal, sphere.getCenter()) + m_distance);
	float distanceFromSphere = distanceFormSphereCenter - sphere.getRadius();
	return intersectData(distanceFromSphere < 1e-6 ,distanceFromSphere);
}
