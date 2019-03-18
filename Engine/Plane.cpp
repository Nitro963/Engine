#include "Plane.h"

plane plane::normalized() const{
	float length = glm::length(m_normal);
	return plane(m_normal/length ,m_distance/length);
}
