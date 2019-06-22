#include "OctTree.h"
#include <exception>

OcTree * OcTree::createNode(const AABB & region, const std::list<RigidBody*>& bodies) {
	if (!bodies.size())
		return nullptr;
	OcTree* ret = new OcTree(region, bodies);
	ret->par = this;
	return ret;
}

OcTree::~OcTree() {
	m_bodies.clear();
	if (m_activationMask)
		for (int i = 0; i < 8; ++i)
			if (m_activationMask & (1 << i))
				delete m_childNode[i];
	m_activationMask = 0;
}

void OcTree::buildTree() {
	//terminate the recursion if we're a leaf node
	if (m_bodies.size() <= 1)
		return;

	glm::vec3 dimensions = m_region.getExtents();
	//Check to see if the dimensions of the box are greater than the minimum dimensions
	if (glm::dot(dimensions - minSize, dimensions - minSize) < EPSILON2)
		return;
	glm::vec3 mn = m_region.getMin();
	glm::vec3 mx = m_region.getMax();
	const glm::vec3& center = m_region.getCenter();

	AABB childrenBoxs[8]{
		AABB(glm::vec4(mn, 0), glm::vec4(center, 0)),
		AABB(glm::vec4(center.x, mn.y, mn.z, 0), glm::vec4(mx.x, center.y, center.z, 0)),
		AABB(glm::vec4(center.x, mn.y, center.z, 0), glm::vec4(mx.x, center.y, mx.z, 0)),
		AABB(glm::vec4(mn.x, mn.y, center.z, 0), glm::vec4(center.x, center.y, mx.z, 0)),
		AABB(glm::vec4(mn.x, center.y, mn.z, 0), glm::vec4(center.x, mx.y, center.z, 0)),
		AABB(glm::vec4(center.x, center.y, mn.z, 0), glm::vec4(mx.x, mx.y, center.z, 0)),
		AABB(glm::vec4(mn.x, center.y, center.z, 0), glm::vec4(center.x, mx.y, mx.z, 0)),
		AABB(glm::vec4(center, 0), glm::vec4(mx, 0))
	};

	std::list<RigidBody*> childrenBodies[8];
	std::vector<RigidBody*> deList;
	for (const auto& body : m_bodies)
		if (body->getCollider1()) {
			for (int i = 0; i < 8; ++i)
				if (childrenBoxs[i].contains(*body->getCollider1())) {
					childrenBodies[i].push_back(body);
					deList.push_back(body);
					break;
				}
		}
		else {
			for (const auto& box : childrenBoxs)
				for (int i = 0; i < 8; ++i)
					if (childrenBoxs[i].contains(*body->getCollider2())) {
						childrenBodies[i].push_back(body);
						deList.push_back(body);
						break;
					}
		}
		for (const auto& body : deList)
			m_bodies.remove(body);

		for (int i = 0; i < 8; ++i)
			if (!childrenBodies[i].empty()) {
				m_childNode[i] = createNode(childrenBoxs[i], childrenBodies[i]);
				m_activationMask |= (1 << i);
				m_childNode[i]->buildTree();
			}
		//m_treeBuilt = 1;
		//m_treeReady = 1;
}

bool OcTree::insert(RigidBody * body) {
	//If the body won't fit into the current region, so it won't fit into any child regions.
	//therefore, try to push it up the tree. If we're at the root node, we need to resize the whole tree.
	if (body->getCollider1()) {
		if (!m_region.contains(*body->getCollider1()))
			return par ? par->insert(body) : false;
	}
	else
		if (!m_region.contains(*body->getCollider2()))
			return par ? par->insert(body) : false;
	//if the current node is an empty leaf node, just insert and leave it.
	if (m_bodies.empty()) {
		m_bodies.push_back(body);
		return true;
	}
	glm::vec3 dimensions = m_region.getExtents();

	//If we're at the smallest size, just insert the item here. We can't go any lower!
	if (glm::dot(dimensions - minSize, dimensions - minSize) < EPSILON2) {
		m_bodies.push_back(body);
		return true;
	}
	glm::vec3 mn = m_region.getMin();
	glm::vec3 mx = m_region.getMax();
	const glm::vec3& center = m_region.getCenter();

	std::list<RigidBody*> childrenBodies[8];

	//Find or create subdivided regions
	AABB childrenBoxs[8];
	childrenBoxs[0] = (m_activationMask & (1 << 0)) ? m_childNode[0]->m_region : AABB(glm::vec4(mn, 0), glm::vec4(center, 0));
	childrenBoxs[1] = (m_activationMask & (1 << 1)) ? m_childNode[1]->m_region : AABB(glm::vec4(center.x, mn.y, mn.z, 0), glm::vec4(mx.x, center.y, center.z, 0));
	childrenBoxs[2] = (m_activationMask & (1 << 2)) ? m_childNode[2]->m_region : AABB(glm::vec4(center.x, mn.y, center.z, 0), glm::vec4(mx.x, center.y, mx.z, 0));
	childrenBoxs[3] = (m_activationMask & (1 << 3)) ? m_childNode[3]->m_region : AABB(glm::vec4(mn.x, mn.y, center.z, 0), glm::vec4(center.x, center.y, mx.z, 0));
	childrenBoxs[4] = (m_activationMask & (1 << 4)) ? m_childNode[4]->m_region : AABB(glm::vec4(mn.x, center.y, mn.z, 0), glm::vec4(center.x, mx.y, center.z, 0));
	childrenBoxs[5] = (m_activationMask & (1 << 5)) ? m_childNode[5]->m_region : AABB(glm::vec4(center.x, center.y, mn.z, 0), glm::vec4(mx.x, mx.y, center.z, 0));
	childrenBoxs[6] = (m_activationMask & (1 << 6)) ? m_childNode[6]->m_region : AABB(glm::vec4(mn.x, center.y, center.z, 0), glm::vec4(center.x, mx.y, mx.z, 0));
	childrenBoxs[7] = (m_activationMask & (1 << 7)) ? m_childNode[7]->m_region : AABB(glm::vec4(center, 0), glm::vec4(mx, 0));

	if (body->getCollider1()) {
		for (int i = 0; i < 8; ++i)
			if (childrenBoxs[i].contains(*body->getCollider1())) {
				if (m_activationMask & (1 << i))
					return m_childNode[i]->insert(body);
				else {
					m_childNode[i] = new OcTree(m_region);
					m_childNode[i]->par = this;
					m_activationMask |= (1 << i);
					return m_childNode[i]->insert(body);
				}
			}
		return m_bodies.push_back(body), true;
	}
	else {
		for (int i = 0; i < 8; ++i)
			if (childrenBoxs[i].contains(*body->getCollider2())) {
				if (m_activationMask & (1 << i))
					return m_childNode[i]->insert(body);
				else {
					m_childNode[i] = new OcTree(m_region);
					m_childNode[i]->par = this;
					m_activationMask |= (1 << i);
					return m_childNode[i]->insert(body);
				}
			}
		return m_bodies.push_back(body), true;
	}
	return false;
}

void OcTree::clear() {
	m_bodies.clear();
	if (m_activationMask)
		for (int i = 0; i < 8; ++i)
			if (m_activationMask & (1 << i))
				delete m_childNode[i];
	m_activationMask = 0;
}

std::vector<ContactData*> OcTree::getContacts(std::list<RigidBody*>& parentList) {
	std::cout << m_bodies.size() << '\n';
	std::vector<ContactData*> ret;

	for (auto it = m_bodies.begin(); it != m_bodies.end(); ++it) {
		auto it1 = it;
		++it1;
		for (; it1 != m_bodies.end(); ++it1) {
			RigidBody* A = *it;
			RigidBody* B = *it1;
			CollisionManifold M;
			bool flip = 0;
			if (A->getCollider1())
				if (B->getCollider1())
					M = A->getCollider1()->findCollisionFeatures(*B->getCollider1());
				else
					M = A->getCollider1()->findCollisionFeatures(*B->getCollider2());
			else
				if (B->getCollider1())
					M = B->getCollider1()->findCollisionFeatures(*A->getCollider2()), flip = 1;
				else
					M = A->getCollider2()->findCollisionFeatures(*B->getCollider2());

			if (M.colliding)
				if (!flip)
					ret.push_back(new ContactData(A, B, new CollisionManifold(M)));
				else
					ret.push_back(new ContactData(B, A, new CollisionManifold(M)));
		}

		for (auto it2 = parentList.begin(); it2 != parentList.end(); ++it2) {
			RigidBody* A = *it;
			RigidBody* B = *it2;
			CollisionManifold M;
			bool flip = 0;
			if (A->getCollider1())
				if (B->getCollider1())
					M = A->getCollider1()->findCollisionFeatures(*B->getCollider1());
				else
					M = A->getCollider1()->findCollisionFeatures(*B->getCollider2());
			else
				if (B->getCollider1())
					M = B->getCollider1()->findCollisionFeatures(*A->getCollider2()), flip = 1;
				else
					M = A->getCollider2()->findCollisionFeatures(*B->getCollider2());

			if (M.colliding)
				if (!flip)
					ret.push_back(new ContactData(A, B, new CollisionManifold(M)));
				else
					ret.push_back(new ContactData(B, A, new CollisionManifold(M)));
		}
	}
	parentList.insert(parentList.end(), m_bodies.begin(), m_bodies.end());
	if (m_activationMask)
		for (int i = 0; i < 8; ++i)
			if (m_activationMask & (1 << i)) {
				std::vector<ContactData*> ve = m_childNode[i]->getContacts(parentList);
				ret.insert(ret.end(), ve.begin(), ve.end());
			}
	return ret;
}

void OcTree::update(const float duration) {
	if (m_bodies.empty()) {
		if (!m_activationMask) {
			if (!(m_curLife + 1))
				m_curLife = m_maxLifeSpan;
			else
				if (m_curLife > 0)
					--m_curLife;
		}
	}
	else
		if (m_curLife + 1) {
			m_maxLifeSpan = glm::min(m_maxLifeSpan * 2, 64);
			m_curLife = -1;
		}
	for (auto& body : m_bodies)
		body->integrate(duration);
	std::vector<RigidBody*> movedBodies;
	for (auto it = m_bodies.begin(); it != m_bodies.end();) {
		RigidBody* body = *it;
		if (body->isDead()) {
			auto tmp = it;
			++it;
			m_bodies.erase(tmp);
			continue;
		}
		else
			++it;
		if (body->isAwake())
			movedBodies.push_back(body);
	}
	//prune out any dead branches in the tree
	if (m_activationMask)
		for (int i = 0; i < 8; ++i)
			if ((m_activationMask & (1 << i)) && !m_childNode[i]->m_curLife) {
				if (m_childNode[i]->m_bodies.size() > 0) {
					throw std::exception("Tried to delete a used branch!");
					m_childNode[i]->m_curLife = -1;
				}
				else {
					delete m_childNode[i];
					m_childNode[i] = nullptr;
					m_activationMask ^= (1 << i);//remove the node from the active nodes mask
				}
			}

	//recursively update any child nodes.
	if (m_activationMask)
		for (int i = 0; i < 8; ++i)
			if (m_activationMask & (1 << i))
				m_childNode[i]->update(duration);
	
	//remove and insert moved bodies
	for (auto& body : movedBodies) {
		m_bodies.remove(body);
		if (!insert(body))
			body->kill();
	}
}
