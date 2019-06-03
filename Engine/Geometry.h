#ifndef GEOMETRY_H
#define GEOMETRY_H

#include"glm\glm.hpp"
#include"glm\gtc\matrix_transform.hpp"
#include"glm\gtc\matrix_inverse.hpp"
#include"glm\gtc\type_ptr.hpp"
#include<vector>

typedef glm::vec3 point;
#define EPSILON 1e-6
#define EPSILON2 1e-12

struct CollisionManifold {
	bool colliding;
	glm::vec3 normal;
	float depth;
	std::vector<point> contacts;

	CollisionManifold() :normal() ,depth(1e9) ,colliding(0) {}
};

struct Line {
	point a, b;
	Line(point a, point b) : a(a), b(b) {}

	inline void closestPoint(const point& c ,float &t, point &d) const {
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
	inline float sqDist(const point& c) const{
		glm::vec3 ab = b - a, ac = c - a, bc = c - b;
		float e = glm::dot(ac, ab);
		// Handle cases where c projects outside ab
		if (e <= 0.0f) return glm::dot(ac, ac);
		float f = glm::dot(ab, ab);
		if (e >= f) return glm::dot(bc, bc);
		// Handle cases where c projects onto ab
		return glm::dot(ac, ac) - e * e / f;
	}
};

struct Interval {
	float mn, mx;
	Interval() : mn(1e9), mx(-1e9) {}
	Interval(const float mn, const float mx) : mn(mn), mx(mx) {}
	bool overlap(const Interval& other) const {
		return (other.mn <= mx) && (mn <= other.mx);
	}
	float getLen() {
		return mx - mn;
	}
};

struct cmpPoint{
	bool operator() (point const & p1, point const & p2) const {
		glm::vec3 u = p2 - p1;
		if (glm::dot(u, u) < EPSILON2)
			return false;
		return p1.x < p2.x;
	}
};

inline glm::mat3 star(const glm::vec3 ve) {
	glm::mat3 m(0.f);
	m[0][1] = -ve.z;
	m[0][2] = ve.y;
	m[1][0] = ve.z;
	m[1][2] = -ve.x;
	m[2][0] = -ve.y;
	m[2][1] = ve.x;
	return m;
}
#endif // !GEOMETRY_H
