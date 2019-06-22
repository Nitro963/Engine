#include "StolenCode.h"
#include "OBB.h"
#include "Geometry.h"
#include "Plane.h"

#include <iostream>
bool PointInOBB(const Point& point, const OBB& obb) {
	vec3 dir = point - obb.c;

	for (int i = 0; i < 3; ++i) {
		const float* orientation = &obb.u[i][0];
		vec3 axis(orientation[0], orientation[1], orientation[2]);

		float distance = glm::dot(dir, axis);

		if (distance > obb.halfExtents[i]) {
			return false;
		}
		if (distance < -obb.halfExtents[i]) {
			return false;
		}
	}

	return true;
}

Interval1 GetInterval(const OBB& obb, const vec3& axis) {
	std::vector<point1> vertex = getVertices(obb);

	Interval1 result;
	for (const auto& v : vertex) {
		result.mn = glm::min(result.mn, glm::dot(axis, v));
		result.mx = glm::max(result.mx, glm::dot(axis, v));
	}

	return result;
}

void ResetCollisionManifold(CollisionManifold* result) {
	if (result != 0) {
		result->colliding = false;
		result->normal = vec3(0, 0, 1);
		result->depth = FLT_MAX;
		if (result->contacts.size() > 0) {
			result->contacts.clear();
		}
	}
}

std::vector<Point> getVertices(const OBB& obb) {
	std::vector<vec3> v;
	v.resize(8);

	vec3 C = obb.c;	// OBB Center
	vec3 E = obb.halfExtents;		// OBB Extents
	const float* o = &obb.u[0][0];
	vec3 A[] = {			// OBB Axis
		vec3(o[0], o[1], o[2]),
		vec3(o[3], o[4], o[5]),
		vec3(o[6], o[7], o[8]),
	};

	v[0] = C + A[0] * E[0] + A[1] * E[1] + A[2] * E[2];
	v[1] = C - A[0] * E[0] + A[1] * E[1] + A[2] * E[2];
	v[2] = C + A[0] * E[0] - A[1] * E[1] + A[2] * E[2];
	v[3] = C + A[0] * E[0] + A[1] * E[1] - A[2] * E[2];
	v[4] = C - A[0] * E[0] - A[1] * E[1] - A[2] * E[2];
	v[5] = C + A[0] * E[0] - A[1] * E[1] - A[2] * E[2];
	v[6] = C - A[0] * E[0] + A[1] * E[1] - A[2] * E[2];
	v[7] = C - A[0] * E[0] - A[1] * E[1] + A[2] * E[2];

	return v;
}

std::vector<Line1> getEdges(const OBB& obb) {
	std::vector<Line1> result;
	result.reserve(12);
	std::vector<Point> vertices = getVertices(obb);

	unsigned int indices[12][2] = { // Indices of edges
		{ 0 ,1 },{ 0 ,3 },{ 0 ,2 },
		{ 1 ,6 },{ 1 ,7 },{ 2 ,7 },
		{ 2 ,5 },{ 3 ,6 },{ 5 ,3 },
		{ 4 ,7 },{ 4 ,6 },{ 4 ,5 }
	};

	for (int i = 0; i < 12;i++)
		result.push_back(Line1(vertices[indices[i][0]], vertices[indices[i][1]]));

	return result;
}

std::vector<plane1> getFaces(const OBB& obb) {
	vec3 c = obb.c;	// OBB Center
	vec3 halfExtents = obb.halfExtents;		// OBB Extents
	const float* o = &obb.u[0][0];
	vec3 u[] = {			// OBB Axis
		vec3(o[0], o[1], o[2]),
		vec3(o[3], o[4], o[5]),
		vec3(o[6], o[7], o[8]),
	};

	std::vector<plane1> result;
	result.resize(6);

	result[0] = plane1(u[0], c + u[0] * halfExtents[0]);
	result[1] = plane1(u[0] * -1.0f, c - u[0] * halfExtents[0]);
	result[2] = plane1(u[1], c + u[1] * halfExtents[1]);
	result[3] = plane1(u[1] * -1.0f, c - u[1] * halfExtents[1]);
	result[4] = plane1(u[2], c + u[2] * halfExtents[2]);
	result[5] = plane1(u[2] * -1.0f, c - u[2] * halfExtents[2]);

	return result;
}

//bool ClipToPlane(const Plane& plane, const Line1& line, Point* outPoint) {
//	vec3 ab = line.end - line.start;
//
//	float nA = glm::dot(plane.normal, line.start);
//	float nAB = glm::dot(plane.normal, ab);
//
//	if (nAB < 0.0000001f) {
//		return false;
//	}
//
//	float t = (plane.distance - nA) / nAB;
//	if (t >= 0.0f && t <= 1.0f) {
//		if (outPoint != 0) {
//			*outPoint = line.start + ab * t;
//		}
//		return true;
//	}
//
//	return false;
//}

std::set<point1, cmpPoint> ClipEdgesToOBB(const std::vector<Line1>& edges, const OBB& obb) {
	std::set<point1, cmpPoint> res;
	point1 intersection;

	std::vector<plane1>& planes = getFaces(obb);

	for (int i = 0; i < planes.size(); i++) 
		for (int j = 0; j < edges.size(); j++) 
			if (planes[i].clip(edges[j], intersection)) 
				if (PointInOBB(intersection, obb)) 
					res.insert(intersection);
	
	return res;
}

float penetrationDepth(const OBB& o1, const OBB& b, const vec3& axis, bool& outShouldFlip) {
	Interval1 i1 = GetInterval(o1 ,glm::normalize(axis));
	Interval1 i2 = GetInterval(b ,glm::normalize(axis));

	if (!i1.overlap(i2))
		return -1; // No penetration

	outShouldFlip = (i2.mn < i1.mn);

	float length = glm::max(i1.mx, i2.mx) - glm::min(i1.mn, i2.mn);

	return (i1.getLen() + i2.getLen()) - length;
}

CollisionManifold FindCollisionFeatures(const OBB& A, const OBB& B) {
	CollisionManifold result; // Will return result of intersection!
	ResetCollisionManifold(&result);


	const float* o1 = &A.u[0][0];
	const float* o2 = &B.u[0][0];

	vec3 test[15] = {
		vec3(o1[0], o1[1], o1[2]),
		vec3(o1[3], o1[4], o1[5]),
		vec3(o1[6], o1[7], o1[8]),
		vec3(o2[0], o2[1], o2[2]),
		vec3(o2[3], o2[4], o2[5]),
		vec3(o2[6], o2[7], o2[8])
	};

	for (int i = 0; i < 3; ++i) { // Fill out rest of axis
		test[6 + i * 3 + 0] = glm::cross(test[i], test[3]);
		test[6 + i * 3 + 1] = glm::cross(test[i], test[4]);
		test[6 + i * 3 + 2] = glm::cross(test[i], test[5]);
	}

	vec3 hitNormal;
	bool shouldFlip;

	for (int i = 0; i < 15; ++i) {
		if (glm::dot(test[i] ,test[i])< EPSILON2)
			continue;

		float depth = penetrationDepth(A, B, test[i], shouldFlip);
		if (depth < 0.0f)
			return result;
		
		else 
			if (depth < result.depth) {
				if (shouldFlip) 
					test[i] = test[i] * -1.0f;
				
				result.depth = depth;
				hitNormal = test[i];
		}
	}

	vec3 axis = glm::normalize(hitNormal);

	std::set<point1, cmpPoint> se1 = ClipEdgesToOBB(getEdges(B), A);
	std::set<point1, cmpPoint> se2 = ClipEdgesToOBB(getEdges(A), B);
	se1.insert(se2.begin(), se2.end());
	
	for (const auto& p : se1)
		result.contacts.push_back(p);
	
	result.colliding = true;
	result.normal = axis;
	return result;
}

OBB::OBB(OBB1 other){
	this->c = other.getCenter();
	this->halfExtents = other.getHalfExtents();
	glm::vec3 u = other.getLocalX();
	glm::vec3 v = other.getLocalY();
	glm::vec3 t = other.getLocalZ();
	this->u[0] = u;
	this->u[1] = v;
	this->u[2] = t;
}
