#include "OBB.h"
#include "BoundingSphere.h"
#include "Plane.h"
#include <set>
std::vector<point> OBB::getVertices() const {
	std::vector<point> ve;

	ve.push_back(c + u[0] * halfExtents[0] + u[1] * halfExtents[1] + u[2] * halfExtents[2]);
	ve.push_back(c - u[0] * halfExtents[0] + u[1] * halfExtents[1] + u[2] * halfExtents[2]);

	ve.push_back(c + u[0] * halfExtents[0] - u[1] * halfExtents[1] + u[2] * halfExtents[2]);
	ve.push_back(c + u[0] * halfExtents[0] + u[1] * halfExtents[1] - u[2] * halfExtents[2]);

	ve.push_back(c - u[0] * halfExtents[0] - u[1] * halfExtents[1] - u[2] * halfExtents[2]);
	ve.push_back(c + u[0] * halfExtents[0] - u[1] * halfExtents[1] - u[2] * halfExtents[2]);

	ve.push_back(c - u[0] * halfExtents[0] + u[1] * halfExtents[1] - u[2] * halfExtents[2]);
	ve.push_back(c - u[0] * halfExtents[0] - u[1] * halfExtents[1] + u[2] * halfExtents[2]);
	return ve;
}

std::vector<Line> OBB::getEdges() const {
	std::vector<point> vertices = getVertices();
	std::vector<Line> ve;
	unsigned int indices[12][2] = {
		{ 0 ,1 },{ 0 ,3 },{ 0 ,2 },
		{ 1 ,6 },{ 1 ,7 },{ 2 ,7 },
		{ 2 ,5 },{ 3 ,6 },{ 5 ,3 },
		{ 4 ,7 },{ 4 ,6 },{ 4 ,5 }
	};
	for (int i = 0; i < 12; i++)
		ve.push_back(Line(vertices[indices[i][0]], vertices[indices[i][1]]));
	return ve;
}

std::vector<plane> OBB::getFaces() const {
	std::vector<plane> ve;
	ve.push_back(plane(u[0], c + u[0] * halfExtents[0]));
	ve.push_back(plane(-u[0], c - u[0] * halfExtents[0]));
	ve.push_back(plane(u[1], c + u[1] * halfExtents[1]));
	ve.push_back(plane(-u[1], c - u[1] * halfExtents[1]));
	ve.push_back(plane(u[2], c + u[2] * halfExtents[2]));
	ve.push_back(plane(-u[2], c - u[2] * halfExtents[2]));
	return ve;
}

std::set<point, cmpPoint> OBB::clipEdges(std::vector<Line>& edges) const {
	std::set<point, cmpPoint> se;
	std::vector<plane> faces = getFaces();
	for (const auto& face : faces) {
		for (const auto& edge : edges) {
			point p;
			if (face.clip(edge, p)) {
				if (containsPoint(p))
					se.insert(p);
			}
		}
	}
	return se;
}

point OBB::closestPoint(const point & p) const {
	glm::vec3 d = p - c;
	// Start result at center of box; make steps from there
	point q = c;
	// For each OBB axis...
	for (int i = 0; i < 3; i++) {
		// ...project d onto that axis to get the distance
		// along the axis of d from the box center
		// If distance farther than the box extents, clamp to the box
		float dist = glm::clamp(glm::dot(d, u[i]), -halfExtents[i], halfExtents[i]);

		// Step that distance along the axis to get world coordinate
		q += dist * u[i];
	}
	return q;
}

bool OBB::containsPoint(const point & p) const {
	for (int i = 0; i < 3; i++) {
		// project p on each axis to get distance
		float d = glm::dot(p, u[i]);
		// check if p project outside the box
		if (d > halfExtents[i])
			return false;
		if (d < -halfExtents[i])
			return false;
	}
	return true;
}

CollisionManifold OBB::findCollisionFeatures(const OBB & b) const {
	CollisionManifold res;
	glm::vec3 test[15] = {
		u[0] ,u[1] ,u[2],b.u[0] ,b.u[1] ,b.u[2]
	};
	for (int i = 0; i < 3; i++) {
		test[6 + 3 * i] = glm::cross(test[i], test[3]);
		test[6 + 3 * i + 1] = glm::cross(test[i], test[4]);
		test[6 + 3 * i + 2] = glm::cross(test[i], test[5]);
	}

	for (const auto& axis : test) {
		bool flip = 0;
		if (glm::dot(axis, axis) < EPSILON2)
			continue;
		float depth = penetrationDepth(b, axis, flip);
		if (depth < EPSILON)
			return res.colliding = false, res;
		if (depth < res.depth) {
			res.depth = depth;
			res.normal = glm::normalize((flip ? -1.f * axis : axis));
		}
	}

	std::set<point, cmpPoint> se1 = clipEdges(b.getEdges());
	std::set<point, cmpPoint> se2 = b.clipEdges(getEdges());
	se1.insert(se2.begin(), se2.end());
	
	Interval i = getInterval(res.normal);
	float distance = (i.mx - i.mn)* 0.5f - res.depth * 0.5f;
	glm::vec3 pointOnPlane = c + res.normal * distance;
	se2.clear();
	for (const auto& p : se1) {
		glm::vec3 pt = p + res.normal * glm::dot(res.normal, pointOnPlane - p);
		se2.insert(pt);
	}
	for (const auto& p : se2)
		res.contacts.push_back(p);
	return res.colliding = true, res;
}

bool OBB::intersect(const OBB & b) const {
	float ra, rb;
	glm::mat3x3 R, AbsR;

	// Compute rotation matrix expressing b in this coordinate frame
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			R[i][j] = glm::dot(u[i], b.getLocalCoord()[j]);

	// Compute translation vector t
	glm::vec3 t = b.getCenter() - c;

	// Bring translation into this coordinate frame
	t = glm::vec3(glm::dot(t, u[0]), glm::dot(t, u[1]), glm::dot(t, u[2]));

	// Compute common subexpressions. Add in an epsilon term to
	// counteract arithmetic errors when two edge are parallel and
	// their cross product is (near) null
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			AbsR[i][j] = glm::abs(R[i][j]) + EPSILON;

	// Test axes L = A0, L = A1, L = A2
	for (int i = 0; i < 3; i++) {
		ra = halfExtents[i];
		rb = b.getXRadius() * AbsR[i][0] + b.getYRadius() * AbsR[i][1] + b.getZRadius() * AbsR[i][2];
		if (glm::abs(t[i]) > ra + rb) return false;

	}

	// Test axes L = B0, L = B1, L = B2
	for (int i = 0; i < 3; i++) {
		ra = halfExtents[0] * AbsR[0][i] + halfExtents[1] * AbsR[1][i] + halfExtents[2] * AbsR[2][i];
		rb = b.getHalfExtents()[i];
		if (glm::abs(t[0] * R[0][i] + t[1] * R[1][i] + t[2] * R[2][i]) > ra + rb) return false;
	}

	// Test axis L = A0 x B0
	ra = halfExtents[1] * AbsR[2][0] + halfExtents[2] * AbsR[1][0];
	rb = b.getYRadius() * AbsR[0][2] + b.getZRadius() * AbsR[0][1];
	if (glm::abs(t[2] * R[1][0] - t[1] * R[2][0]) > ra + rb) return false;

	// Test axis L = A0 x B1
	ra = halfExtents[1] * AbsR[2][1] + halfExtents[2] * AbsR[1][1];
	rb = b.getXRadius() * AbsR[0][2] + b.getZRadius() * AbsR[0][0];
	if (glm::abs(t[2] * R[1][1] - t[1] * R[2][1]) > ra + rb) return false;

	// Test axis L = A0 x B2
	ra = halfExtents[1] * AbsR[2][2] + halfExtents[2] * AbsR[1][2];
	rb = b.getXRadius() * AbsR[0][1] + b.getYRadius() * AbsR[0][0];
	if (glm::abs(t[2] * R[1][2] - t[1] * R[2][2]) > ra + rb) return false;

	// Test axis L = A1 x B0
	ra = halfExtents[0] * AbsR[2][0] + halfExtents[2] * AbsR[0][0];
	rb = b.getYRadius() * AbsR[1][2] + b.getZRadius() * AbsR[1][1];
	if (glm::abs(t[0] * R[2][0] - t[2] * R[0][0]) > ra + rb) return false;

	// Test axis L = A1 x B1
	ra = halfExtents[0] * AbsR[2][1] + halfExtents[2] * AbsR[0][1];
	rb = b.getXRadius() * AbsR[1][2] + b.getZRadius() * AbsR[1][0];
	if (glm::abs(t[0] * R[2][1] - t[2] * R[0][1]) > ra + rb) return false;

	// Test axis L = A1 x B2
	ra = halfExtents[0] * AbsR[2][2] + halfExtents[2] * AbsR[0][2];
	rb = b.getXRadius() * AbsR[1][1] + b.getYRadius() * AbsR[1][0];
	if (glm::abs(t[0] * R[2][2] - t[2] * R[0][2]) > ra + rb) return false;

	// Test axis L = A2 x B0
	ra = halfExtents[0] * AbsR[1][0] + halfExtents[1] * AbsR[0][0];
	rb = b.getYRadius() * AbsR[2][2] + b.getZRadius() * AbsR[2][1];
	if (glm::abs(t[1] * R[0][0] - t[0] * R[1][0]) > ra + rb) return false;

	// Test axis L = A2 x B1
	ra = halfExtents[0] * AbsR[1][1] + halfExtents[1] * AbsR[0][1];
	rb = b.getXRadius() * AbsR[2][2] + b.getZRadius() * AbsR[2][0];
	if (glm::abs(t[1] * R[0][1] - t[0] * R[1][1]) > ra + rb) return false;

	// Test axis L = A2 x B2
	ra = halfExtents[0] * AbsR[1][2] + halfExtents[1] * AbsR[0][2];
	rb = b.getXRadius() * AbsR[2][1] + b.getYRadius() * AbsR[2][0];
	if (glm::abs(t[1] * R[0][2] - t[0] * R[1][2]) > ra + rb) return false;

	// Since no separating axis is found, the OBBs must be intersecting
	return true;
}

bool OBB::intersect(const boundingSphere & b) const {
	return b.intersect(*this);
}

void OBB::sync(const point & c, const glm::mat3& rotationT) {
	this->c = c;
	this->u[0] = rotationT[0]; this->u[1] = rotationT[1]; this->u[2] = rotationT[2];
}

void OBB::update(const point & c, const glm::mat3& rotationT, const glm::vec3 & edge) {
	this->c = c;
	this->u[0] = rotationT[0]; this->u[1] = rotationT[1]; this->u[2] = rotationT[2];
	this->halfExtents = edge;
}

Interval OBB::getInterval(const glm::vec3& axis) const {
	std::vector<point> vertices = getVertices();
	Interval res;
	for (const auto& v : vertices) {
		//project each vertex on the given axis
		float projection = glm::dot(v, axis);
		//save minimum and maximum projection to get interval
		res.mn = glm::min(res.mn, projection);
		res.mx = glm::max(res.mx, projection);
	}
	return res;
}

float OBB::penetrationDepth(const OBB & b, const glm::vec3 & axis, bool & flip) const {
	Interval i1 = getInterval(axis);
	Interval i2 = b.getInterval(axis);
	if (!i1.overlap(i2))
		return 0.0f;
	float max = glm::max(i1.mx, i2.mx);
	float min = glm::min(i1.mn, i2.mn);
	float length = max - min;
	flip = (i2.mn < i1.mn);
	return i1.getLen() + i2.getLen() - length;
}
