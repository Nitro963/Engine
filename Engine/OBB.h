#ifndef OBB_H
#define OBB_H
#include"Geometry.h"
class OBB {
public:
	//Given OBB center ,local space axes and three scalers
	OBB(point& c, glm::vec3 u[3], glm::vec3& edge) :c(c), halfExtents(edge) { this->u[0] = glm::vec3(u[0]); this->u[1] = glm::vec3(u[1]); this->u[2] = glm::vec3(u[2]); }
	OBB(const point& c, const glm::mat3& rotationT, const glm::vec3& edge) :c(c), halfExtents(edge) { this->u[0] = rotationT[0]; this->u[1] = rotationT[1]; this->u[2] = rotationT[2]; }
	
	inline const point& getCenter() const { return c; }
	inline const glm::vec3& getLocalX() const { return u[0]; }
	inline const glm::vec3& getLocalY() const { return u[1]; }
	inline const glm::vec3& getLocalZ() const { return u[2]; }
	inline const float getXRadius() const { return halfExtents.x; }
	inline const float getYRadius() const { return halfExtents.y; }
	inline const float getZRadius() const { return halfExtents.z; }
	inline const glm::vec3* getLocalCoord() const { return u; }
	inline const glm::vec3& getHalfExtents() const { return halfExtents; }

	// Given point p, return the point q on or in the OBB that is closest to p
	point closestPoint(const point& p) const;

	// Determine whether the OBB intersects OBB b
	bool testOBB(const OBB& b) const;
private:
	point c; //OBB center
	glm::vec3 u[3]; //OBB local axis
	glm::vec3 halfExtents; // positive halfwidth extents
};
#endif // !OBB_H
