#include "Ray.h"
#include "Plane.h"

bool Ray::intersect(const BoundingSphere & sphere, float& ret) const {
	glm::vec3 m = p - sphere.c;
	float c = glm::dot(m, m) - sphere.r * sphere.r;
	float b = glm::dot(m, d);
	if (b > 0.f && c > 0.f)
		return false;
	float delta = b * b - c;
	if (delta < EPSILON)
		return false;
	float t = -b - glm::sqrt(delta);
	if (t < EPSILON)
		t = 0.f;
	ret = t;
	return true;
}

bool Ray::intersect(const AABB & box, float & ret) const {
	float tmin = 0.f;
	float tmax = 1e9;
	for (int i = 0; i < 3; i++) {
		if (abs(d[i]) < EPSILON)
			if (p[i] < box.mn[i] || p[i] > box.mx[i])
				return false;
		float t1 = (box.mn[i] - p[i]) / d[i];
		float t2 = (box.mx[i] - p[i]) / d[i];
		if (t1 > t2)
			std::swap(t1, t2);
		if (t1 > tmin) tmin = t1;

		if (t2 < tmax) tmax = t2;

		if (tmin > tmax)
			return false;
	}
	ret = tmin;
	return true;
}

bool Ray::intersect(const OBB & box, float& ret) const {
	std::vector<Plane> faces = box.getFaces();
	float tmin = 0;
	float tmax = 1e9;
	for (int i = 0; i < 3; i++) {
		float t1;
		float t2;
		if (intersect(faces[2 * i], t1) && intersect(faces[2 * i + 1], t2)) {
			if (t1 > t2)
				std::swap(t1, t2);
			if (t1 > tmin) tmin = t1;

			if (t2 < tmax) tmax = t2;

			if (tmin > tmax)
				return false;
		}
	}
	if (!box.contains(this->p + tmin * d))
		return false;
	ret = tmin;
	return true;
}

bool Ray::intersect(const Plane& p, float& ans) const {
	// Compute the t value for the directed ray intersecting the plane
	float t = (p.d - glm::dot(p.n, this->p)) / glm::dot(d, p.n);
	if (t > 0.0f) {
		ans = t;
		return true;
	}
	// Else no intersection
	return false;
}
