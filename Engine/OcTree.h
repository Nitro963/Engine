#ifndef OCTTREE_H
#define OCTTREE_H
#include "AABB.h"
#include "RigidBody.h"
#include <queue>
#include <vector>
#include <list>
#include "RigidBody.h"

class OcTree {
private:
	AABB m_region;
	std::list<RigidBody*> m_bodies;
	OcTree* m_childNode[8];
	unsigned int m_activationMask = 0;
	static const glm::vec3 minSize;
	int m_maxLifeSpan = 8;
	int m_curLife = -1;
	OcTree* par;
public:
	OcTree() : m_region(glm::vec3(0), minSize) {};
	OcTree(const AABB& region, const std::list<RigidBody*>& bodies) : m_region(region), m_bodies(bodies) {}
	OcTree(const AABB& region) : m_region(region) {}
	OcTree(const glm::vec3& center, const float d) : m_region(center, glm::vec3(d)) {}
	~OcTree();
	bool insert(RigidBody* body);
	bool dfs();
	void clear();
	std::vector<ContactData*> getContacts(std::list<RigidBody*> parentList);
	void update(const float duration);
};
#endif // !OCTTREE_H
