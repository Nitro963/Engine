#ifndef GEOMETRY_H
#define GEOMETRY_H

#include"glm\glm.hpp"
#include"glm\gtc\matrix_transform.hpp"
#include"glm\gtc\matrix_inverse.hpp"
#include"glm\gtc\type_ptr.hpp"

typedef glm::vec3 point;
#define EPSILON 1e-6
#define EPSILON2 1e-3

struct CollisionData {
	point collisionPoint;
	glm::vec3 contactNormal;
	float penetrationDepth;

	CollisionData(const point& collisionPoint ,const glm::vec3 contactNormal ,float penetrationDepth) : collisionPoint(collisionPoint) ,contactNormal(contactNormal) ,penetrationDepth(penetrationDepth){}
	CollisionData(){}
};

inline void closestPointSegment(const point& c, const point& a, const point& b, float &t, point &d){
	glm::vec3 ab = b - a;

	// Project c onto ab, but deferring divide by Dot(ab, ab)
	t = glm::dot(c - a, ab);
	if (t <= 0.0f) {
		// c projects outside the [a,b] interval, on the a side; clamp to a
		t = 0.0f;
		d = a;
	}
	else {
		float denom = glm::dot(ab, ab); // Always nonnegative since denom = ||ab||∧2
		if (t >= denom) {
			// c projects outside the [a,b] interval, on the b side; clamp to b
			t = 1.0f;
			d = b;
		}
		else {
			// c projects inside the [a,b] interval; must do deferred divide now
			t = t / denom;
			d = a + t * ab;
		}
	}
}

// Returns the squared distance between point c and segment ab
inline float sqDistPointSegment(const point& a, const point& b, const point& c){
	glm::vec3 ab = b - a, ac = c - a, bc = c - b;
	float e = glm::dot(ac, ab);
	// Handle cases where c projects outside ab
	if (e <= 0.0f) return glm::dot(ac, ac);
	float f = glm::dot(ab, ab);
	if (e >= f) return glm::dot(bc, bc);
	// Handle cases where c projects onto ab
	return glm::dot(ac, ac) - e * e / f;
}


#endif // !GEOMETRY_H
