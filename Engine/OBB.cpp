#include "OBB.h"

point OBB::closestPoint(const point & p) const{
	glm::vec3 d = p - c;
	// Start result at center of box; make steps from there
	point q = c;
	// For each OBB axis...
	for (int i = 0; i < 3; i++) {
		// ...project d onto that axis to get the distance
		// along the axis of d from the box center
		// If distance farther than the box extents, clamp to the box
		float dist = glm::clamp(glm::dot(d, u[i]) ,-halfExtents[i] ,halfExtents[i]);
		
		// Step that distance along the axis to get world coordinate
		q += dist * u[i];
	}
	return q;
}

bool OBB::testOBB(const OBB & b) const {
	float ra, rb;
	glm::mat3x3 R, AbsR;

	// Compute rotation matrix expressing b in a’s coordinate frame
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			R[i][j] = glm::dot(u[i], b.getLocalCoord()[j]);

	// Compute translation vector t
	glm::vec3 t = b.getCenter() - c;

	// Bring translation into a’s coordinate frame
	t = glm::vec3(glm::dot(t, u[0]), glm::dot(t, u[2]), glm::dot(t, u[2]));

	// Compute common subexpressions. Add in an epsilon term to
	// counteract arithmetic errors when two edge are parallel and
	// their cross product is (near) null (see text for details)
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
