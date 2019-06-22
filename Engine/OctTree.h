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
	static std::queue<RigidBody*> m_pendingInsertion;
	OcTree* m_childNode[10];
	unsigned int m_activationMask = 0;
	const glm::vec3 minSize = glm::vec3(1.f ,1.f ,1.f);
	int m_maxLifeSpan = 8;
	int m_curLife = -1;
	OcTree* par;
	OcTree* createNode(const AABB& region, const std::list<RigidBody*>& bodies);
public:
	//bool m_treeReady;
	//bool m_treeBuilt;
	OcTree() : m_region(glm::vec3(0), minSize) {};
	OcTree(const AABB& region, const std::list<RigidBody*>& bodies) : m_region(region), m_bodies(bodies) {}
	OcTree(const AABB& region) : m_region(region) {}
	OcTree(const glm::vec3& center, const float d) : m_region(center, glm::vec3(d)) {}
	~OcTree();
	void buildTree();
	bool insert(RigidBody* body);
	void clear();
	std::vector<ContactData*> getContacts(std::list<RigidBody*>& parentList);
	void update(const float duration);
};
#endif // !OCTTREE_H
