#ifndef SHEADER
#define SHEADER

#include <vector>
#include <utility>
//#include <cfloat>
#include <ostream>
#include "glm\glm.hpp"
#include <set>
using namespace glm;
typedef vec3 Point;

typedef struct Line {
	Point start;
	Point end;

	inline Line() {}
	inline Line(const Point& s, const Point& e) :
		start(s), end(e) { }
} Line;

class OBB1;
typedef struct OBB {
	Point c;
	vec3 halfExtents; // HALF SIZE!
	glm::vec3 u[3];

	OBB(OBB1 other);
} OBB;

typedef struct Plane {
	vec3 normal;
	float distance;

	inline Plane() : normal(1, 0, 0) { }
	inline Plane(const vec3& n, float d) :
		normal(n), distance(d) { }
} Plane;


typedef struct Interval {
	float min;
	float max;
} Interval;

bool PointInOBB(const Point& point, const OBB& obb);
struct Interval1;
Interval1 GetInterval(const OBB& obb, const vec3& axis);

// Chapter 15

typedef struct CollisionManifold {
	bool colliding;
	vec3 normal;
	float depth;
	std::vector<vec3> contacts;
};
void ResetCollisionManifold(CollisionManifold* result);
typedef glm::vec3 point1;
std::vector<point1> getVertices(const OBB& obb);
struct Line1;
std::vector<Line1> getEdges(const OBB& obb);
class plane1;
std::vector<plane1> getFaces(const OBB& obb);
//bool ClipToPlane(const Plane& plane, const Line& line, Point* outPoint);
struct cmpPoint;
std::set<point1 ,cmpPoint> ClipEdgesToOBB(const std::vector<Line1>& edges, const OBB& obb);
float penetrationDepth(const OBB& o1, const OBB& b, const vec3& axis, bool& outShouldFlip);

CollisionManifold FindCollisionFeatures(const OBB& A, const OBB& B);
#endif // !SHEADER
