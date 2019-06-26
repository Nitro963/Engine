#include "OBB.h"
#include "BoundingSphere.h"
#include "Plane.h"

std::vector<point> OBB::getVertices() const{
	std::vector<glm::vec3> ve;
	ve.reserve(8);
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

std::vector<Line> OBB::getEdges() const{
	std::vector<Line> res;
	std::vector<point> vertices = getVertices();
	res.reserve(12);
	unsigned int indices[12][2] = {
		{ 0, 1 },{ 0, 3 },{ 0, 2 },
		{ 1, 6 },{ 1, 7 },{ 2, 7 },
		{ 2, 5 },{ 5, 3 },{ 3, 6 },
		{ 4, 7 },{ 4, 6 },{ 4, 5 }
	};

	for (int i = 0; i < 12; i++)
		res.push_back(Line(vertices[indices[i][0]], vertices[indices[i][1]]));

	return res;
}

std::vector<Plane> OBB::getFaces() const{

	std::vector<Plane> ve;
	ve.reserve(6);
	ve.push_back(Plane(u[0], c + u[0] * halfExtents[0]));
	ve.push_back(Plane(-u[0], c - u[0] * halfExtents[0]));

	ve.push_back(Plane(u[1], c + u[1] * halfExtents[1]));
	ve.push_back(Plane(-u[1], c - u[1] * halfExtents[1]));
	
	ve.push_back(Plane(u[2], c + u[2] * halfExtents[2]));
	ve.push_back(Plane(-u[2], c - u[2] * halfExtents[2]));

	return ve;
}

std::set<point, cmpPoint> OBB::clipEdges(const std::vector<Line>& edges) const{

	std::set<point, cmpPoint> res;
	point intersection = point(0, 0, 0);
	
	std::vector<Plane>& planes = getFaces();
	for (int i = 0; i < planes.size(); i++)
		for (int j = 0; j < edges.size(); j++)
			if (planes[i].clip(edges[j], intersection))//clip the edge to face plane
				if (contains(intersection))//check if the intersection point is in or on the obb
					res.insert(intersection);
					
	return res;
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

CollisionManifold OBB::findCollisionFeatures(const OBB & b) const {
	CollisionManifold res; // Will return intersection Data!

	glm::vec3 axis[15] = {
		u[0], u[1], u[2],
		b.u[0], b.u[1], b.u[2]
	};

	for (int i = 0; i < 3; ++i) {
		axis[6 + i * 3] = glm::cross(axis[i], axis[3]);
		axis[6 + i * 3 + 1] = glm::cross(axis[i], axis[4]);
		axis[6 + i * 3 + 2] = glm::cross(axis[i], axis[5]);
	}

	for (int i = 0; i < 15; ++i) {
		bool shouldFlip = 0;
		// check for if the axis is degenerated
		if (glm::dot(axis[i], axis[i])< EPSILON2)
			continue;
		axis[i] = glm::normalize(axis[i]);
		float depth = separationDistance(b, axis[i], shouldFlip);
		if (depth < 0.0f)
			return res;// a separation axis is found
		else
			if (depth < res.depth) {//keep track of the axis with the least amount of separation
				res.depth = depth;
				res.normal = axis[i] * (shouldFlip ? -1.f : 1.f);
			}
	}
	//get contact points without duplication(using Set data structure)
	std::set<point, cmpPoint> se1 = clipEdges(b.getEdges());
	std::set<point, cmpPoint> se2 = b.clipEdges(getEdges());	
	se1.insert(se2.begin(), se2.end());
	res.contacts.reserve(se1.size());
	//copy the contact points to the CollisionManifold
	for (const auto& p : se1)
		res.contacts.push_back(p);

	return res.colliding = true, res;
}

bool OBB::intersect(const OBB & b) const {
	float ra, rb;
	glm::mat3x3 R, AbsR;

	// Compute rotation matrix expressing b in this coordinate frame
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			R[i][j] = glm::dot(u[i], b.u[j]);

	// Compute translation vector t
	glm::vec3 t = b.c - c;

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
		rb = b.halfExtents.x * AbsR[i][0] + b.halfExtents.y * AbsR[i][1] + b.halfExtents.z * AbsR[i][2];
		if (glm::abs(t[i]) > ra + rb) return false;

	}

	// Test axes L = B0, L = B1, L = B2
	for (int i = 0; i < 3; i++) {
		ra = halfExtents[0] * AbsR[0][i] + halfExtents[1] * AbsR[1][i] + halfExtents[2] * AbsR[2][i];
		rb = b.halfExtents[i];
		if (glm::abs(t[0] * R[0][i] + t[1] * R[1][i] + t[2] * R[2][i]) > ra + rb) return false;
	}

	// Test axis L = A0 x B0
	ra = halfExtents[1] * AbsR[2][0] + halfExtents[2] * AbsR[1][0];
	rb = b.halfExtents.y * AbsR[0][2] + b.halfExtents.z * AbsR[0][1];
	if (glm::abs(t[2] * R[1][0] - t[1] * R[2][0]) > ra + rb) return false;

	// Test axis L = A0 x B1
	ra = halfExtents[1] * AbsR[2][1] + halfExtents[2] * AbsR[1][1];
	rb = b.halfExtents.x * AbsR[0][2] + b.halfExtents.z * AbsR[0][0];
	if (glm::abs(t[2] * R[1][1] - t[1] * R[2][1]) > ra + rb) return false;

	// Test axis L = A0 x B2
	ra = halfExtents[1] * AbsR[2][2] + halfExtents[2] * AbsR[1][2];
	rb = b.halfExtents.x * AbsR[0][1] + b.halfExtents.y * AbsR[0][0];
	if (glm::abs(t[2] * R[1][2] - t[1] * R[2][2]) > ra + rb) return false;

	// Test axis L = A1 x B0
	ra = halfExtents[0] * AbsR[2][0] + halfExtents[2] * AbsR[0][0];
	rb = b.halfExtents.y * AbsR[1][2] + b.halfExtents.z * AbsR[1][1];
	if (glm::abs(t[0] * R[2][0] - t[2] * R[0][0]) > ra + rb) return false;

	// Test axis L = A1 x B1
	ra = halfExtents[0] * AbsR[2][1] + halfExtents[2] * AbsR[0][1];
	rb = b.halfExtents.x * AbsR[1][2] + b.halfExtents.z * AbsR[1][0];
	if (glm::abs(t[0] * R[2][1] - t[2] * R[0][1]) > ra + rb) return false;

	// Test axis L = A1 x B2
	ra = halfExtents[0] * AbsR[2][2] + halfExtents[2] * AbsR[0][2];
	rb = b.halfExtents.x * AbsR[1][1] + b.halfExtents.y * AbsR[1][0];
	if (glm::abs(t[0] * R[2][2] - t[2] * R[0][2]) > ra + rb) return false;

	// Test axis L = A2 x B0
	ra = halfExtents[0] * AbsR[1][0] + halfExtents[1] * AbsR[0][0];
	rb = b.halfExtents.y * AbsR[2][2] + b.halfExtents.z * AbsR[2][1];
	if (glm::abs(t[1] * R[0][0] - t[0] * R[1][0]) > ra + rb) return false;

	// Test axis L = A2 x B1
	ra = halfExtents[0] * AbsR[1][1] + halfExtents[1] * AbsR[0][1];
	rb = b.halfExtents.x * AbsR[2][2] + b.halfExtents.z * AbsR[2][0];
	if (glm::abs(t[1] * R[0][1] - t[0] * R[1][1]) > ra + rb) return false;

	// Test axis L = A2 x B2
	ra = halfExtents[0] * AbsR[1][2] + halfExtents[1] * AbsR[0][2];
	rb = b.halfExtents.x * AbsR[2][1] + b.halfExtents.y * AbsR[2][0];
	if (glm::abs(t[1] * R[0][2] - t[0] * R[1][2]) > ra + rb) return false;

	// Since no separating axis is found, the OBBs must be intersecting
	return true;
}

bool OBB::contains(const point& p) const {
	glm::vec3 q = closestPoint(p);
	q = q - p;
	return dot(q, q) < EPSILON2;
}

Interval OBB::getInterval(const glm::vec3& axis) const{
		std::vector<point> vertices = getVertices();
		Interval res;
		for (const auto& v : vertices) {
			res.mn = glm::min(res.mn, glm::dot(axis, v));
			res.mx = glm::max(res.mx, glm::dot(axis, v));
		}
		return res;	
}

float OBB::separationDistance(const OBB& b, const glm::vec3& axis, bool& outShouldFlip) const{
	//project the obb's onto the axis
	Interval i1 = getInterval(axis);
	Interval i2 = b.getInterval(axis);

	if (!i1.overlap(i2))
		return -1; // No penetration
	
	outShouldFlip = (i2.mn < i1.mn);//check if b is infront the obb

	float length = glm::max(i1.mx, i2.mx) - glm::min(i1.mn, i2.mn);

	return (i1.getLen() + i2.getLen()) - length;
}