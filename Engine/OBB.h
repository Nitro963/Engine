#ifndef OBB_H
#define OBB_H
#include"Geometry.h"

#include<vector>
#include<set>

class boundingSphere;
class plane;
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
	
	//return the set of vertices of the obb
	std::vector<point> getVertices() const;

	//return the set of edges of the obb
	std::vector<Line> getEdges() const;
	
	//return the set of faces of the obb
	std::vector<plane> getFaces() const;

	// clip the edges to the faces of the obb
	std::set<point ,cmpPoint> clipEdges(std::vector<Line>& edges) const;

	// Given point p, return the point q on or in the OBB that is closest to p
	point closestPoint(const point& p) const;
	
	// Given point p, check whether it is on or in the obb
	bool containsPoint(const point& p) const;
	
	//generate contacts
	CollisionManifold findCollisionFeatures(const OBB& b) const;

	// Determine whether the OBB intersects OBB b
	bool intersect(const OBB& b) const;

	// Determine whether the OBB intersects Sphere b
	bool intersect(const boundingSphere& b) const;

	void sync(const point& c, const glm::mat3& rotationT);
	void update(const point& c, const glm::mat3& rotationT ,const glm::vec3& edge);
private:
	point c; //OBB center
	glm::vec3 u[3]; //OBB local axis
	glm::vec3 halfExtents; // positive halfwidth extents

	//project the obb onto the given axis
	Interval getInterval(const glm::vec3& axis) const;
	//calculate pentrationDepth on the given axis
	float penetrationDepth(const OBB& b,const glm::vec3& axis, bool& flip) const;
};
#endif // !OBB_H
