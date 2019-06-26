#ifndef OBB_H
#define OBB_H
#include"Geometry.h"

#include<vector>
#include<set>
#include "AABB.h"


class OBB {
public:
	//Given OBB center, local space axes and three scalers
	OBB(point& c, glm::vec3 u[3], glm::vec3& halfExtents) :c(c), halfExtents(halfExtents), u{ u[0], u[1], u[2] } {}
	OBB(const point& c, const glm::mat3& rotation, const glm::vec3& edge) :c(c), halfExtents(edge), u(rotation) {}
	//casting AABB into OBB
	OBB(const AABB& box) : c(box.getCenter()), halfExtents(box.getHalfExtents()), u{ glm::vec3(1, 0, 0),  glm::vec3(0, 1, 0), glm::vec3(0, 0, 1) } {};
	inline void sync(const point& c, const glm::mat3& rotation) {
		this->c = c;
		this->u[0] = rotation[0]; this->u[1] = rotation[1]; this->u[2] = rotation[2];
	};
	inline void update(const point& c, const glm::mat3& rotation, const glm::vec3& halfExtents) {
		this->c = c;
		this->u[0] = rotation[0]; this->u[1] = rotation[1]; this->u[2] = rotation[2];
		this->halfExtents = halfExtents;
	};
	//return the set of vertices of the obb
	std::vector<point> getVertices() const;

	//return the set of edges of the obb
	std::vector<Line> getEdges() const;

	//return the set of faces of the obb
	std::vector<Plane> getFaces() const;

	// Given point p, return the point q on or in the OBB that is closest to p
	point closestPoint(const point& p) const;

	// generate contacts
	CollisionManifold OBB::findCollisionFeatures(const OBB & b) const;

	// Given point p, check whether it is on or in the obb
	bool contains(const point & p) const;

	//project the obb onto the given axis
	Interval getInterval(const glm::vec3& axis) const;

	// Determine whether the OBB intersects OBB b
	bool intersect(const OBB& b) const;
private:
	point c; //OBB center
	glm::mat3x3 u; //OBB local axis
	glm::vec3 halfExtents; // positive halfwidth extents
	
    //calculate separationDistance on the given axis
	float separationDistance(const OBB& b, const glm::vec3& axis, bool& outShouldFlip) const;

	// clip the edges to the planes of the obb
	std::set<point, cmpPoint> clipEdges(const std::vector<Line>& edges) const;

	friend class Plane;
	friend class BoundingSphere;
	friend class Ray;
	friend class AABB;
};
#endif // !OBB_H
