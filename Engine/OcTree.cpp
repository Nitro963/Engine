#include "OcTree.h"
#include <exception>

const glm::vec3 OcTree::minSize = glm::vec3(1.f);

OcTree::~OcTree() {
	std::cout << "deleting a node\n";
	m_bodies.clear();
	for (int i = 0; i < 8; ++i)
		if (m_childNode[i])
			delete m_childNode[i];
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
	if (m_bodies.empty() && !m_activationMask) {
		m_bodies.push_back(body);
		return true;
	}
	glm::vec3 dimensions = m_region.getExtents();

	//If we're at the smallest size, just insert the body here. We can't go any lower!
	if (glm::dot(dimensions - minSize, dimensions - minSize) < EPSILON2) {
		m_bodies.push_back(body);
		return true;
	}
	const glm::vec3& mn = m_region.getMin();
	const glm::vec3& mx = m_region.getMax();
	const glm::vec3& center = m_region.getCenter();

	std::list<RigidBody*> childrenBodies[8];

	//Find or create subdivided regions
	AABB childrenBoxs[8];
	childrenBoxs[0] = (m_childNode[0]) ? m_childNode[0]->m_region : AABB(glm::vec4(mn, 0), glm::vec4(center, 0));
	childrenBoxs[1] = (m_childNode[1]) ? m_childNode[1]->m_region : AABB(glm::vec4(center.x, mn.y, mn.z, 0), glm::vec4(mx.x, center.y, center.z, 0));
	childrenBoxs[2] = (m_childNode[2]) ? m_childNode[2]->m_region : AABB(glm::vec4(center.x, mn.y, center.z, 0), glm::vec4(mx.x, center.y, mx.z, 0));
	childrenBoxs[3] = (m_childNode[3]) ? m_childNode[3]->m_region : AABB(glm::vec4(mn.x, mn.y, center.z, 0), glm::vec4(center.x, center.y, mx.z, 0));
	childrenBoxs[4] = (m_childNode[4]) ? m_childNode[4]->m_region : AABB(glm::vec4(mn.x, center.y, mn.z, 0), glm::vec4(center.x, mx.y, center.z, 0));
	childrenBoxs[5] = (m_childNode[5]) ? m_childNode[5]->m_region : AABB(glm::vec4(center.x, center.y, mn.z, 0), glm::vec4(mx.x, mx.y, center.z, 0));
	childrenBoxs[6] = (m_childNode[6]) ? m_childNode[6]->m_region : AABB(glm::vec4(mn.x, center.y, center.z, 0), glm::vec4(center.x, mx.y, mx.z, 0));
	childrenBoxs[7] = (m_childNode[7]) ? m_childNode[7]->m_region : AABB(glm::vec4(center, 0), glm::vec4(mx, 0));

	if (m_curLife + 1) {
		m_curLife = -1;
		m_maxLifeSpan = glm::min(m_maxLifeSpan * 2, 64);
	}

	if (body->getCollider1()) {
		for (int i = 0; i < 8; ++i)
			if (childrenBoxs[i].contains(*body->getCollider1())) {
				if (m_childNode[i])
					return m_childNode[i]->insert(body);
				else {
					m_childNode[i] = new OcTree(childrenBoxs[i]);
					m_childNode[i]->par = this;
					return m_childNode[i]->insert(body);
				}
			}
		return m_bodies.push_back(body), true;
	}
	else {
		for (int i = 0; i < 8; ++i)
			if (childrenBoxs[i].contains(*body->getCollider2())) {
				if (m_childNode[i])
					return m_childNode[i]->insert(body);
				else {
					m_childNode[i] = new OcTree(childrenBoxs[i]);
					m_childNode[i]->par = this;
					return m_childNode[i]->insert(body);
				}
			}
		return m_bodies.push_back(body), true;
	}
	return false;
}

bool OcTree::dfs(){
	m_activationMask = 0;
	for (int i = 0; i < 8; i++)
		if (m_childNode[i])
			if (m_childNode[i]->dfs())
				m_activationMask |= (1 << i);
	if (m_bodies.empty() && !m_activationMask) {
		if (m_curLife > 0)
			--m_curLife;
		else
			m_curLife = m_maxLifeSpan;
		return false;
	}
	else {
		if (m_curLife + 1) {
			m_curLife = -1;
			m_maxLifeSpan = glm::min(m_maxLifeSpan * 2, 64);
		}
		return true;
	}
}

void OcTree::clear() {
	m_bodies.clear();
	for (int i = 0; i < 8; ++i)
		if (m_childNode[i]) {
			delete m_childNode[i];
			m_childNode[i] = nullptr;
		}
}

std::vector<ContactData*> OcTree::getContacts(std::list<RigidBody*> parentList) {
	std::vector<ContactData*> ret;
	for (auto it = m_bodies.rbegin(); it != m_bodies.rend(); ++it) {
		for (auto it1 = m_bodies.begin(); *it1 != *it; ++it1) {
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

		for (auto it1 = parentList.begin(); it1 != parentList.end(); ++it1) {
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
	}
	parentList.insert(parentList.end(), m_bodies.begin(), m_bodies.end());
	for (int i = 0 ,flag = m_activationMask; flag; ++i, flag >>= 1)
		if (flag & 1) {
			std::vector<ContactData*> ve = m_childNode[i]->getContacts(parentList);
			ret.insert(ret.end(), ve.begin(), ve.end());
		}
	return ret;
}

void OcTree::update(const float duration) {
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
	for (int i = 0; i < 8; ++i)
		if (m_childNode[i] && !m_childNode[i]->m_curLife) {
			if (m_childNode[i]->m_bodies.size() > 0) {
				throw std::exception("Tried to delete a used branch!");
				m_childNode[i]->m_curLife = -1;
			}
			else {
				delete m_childNode[i];
				m_childNode[i] = nullptr;
				m_activationMask &= ~(1 << i);
			}
		}

	//recursively update any child nodes.
	for (int i = 0; i < 8; ++i)
		if (m_childNode[i])
			m_childNode[i]->update(duration);

	//remove and reinsert moved bodies
	for (auto& body : movedBodies) {
		m_bodies.remove(body);
		if (!insert(body))
			body->kill();
	}
}

void OcTree::getNearest(const Ray & r, RigidBody *& body, float & t) const{
	float ret;
	for (int i = 0, flag = m_activationMask; flag; ++i, flag >>= 1)
		if (flag & 1)
			if (r.intersect(m_childNode[i]->m_region, ret))
				m_childNode[i]->getNearest(r, body, t);
	for (auto& b : m_bodies) {
		if (b->getCollider1()) {
			if (r.intersect(*b->getCollider1(), ret))
				if (ret < t){
					t = ret;
					body = b;
				}
		}
		else
			if(r.intersect(*b->getCollider2(),ret))
				if (ret < t) {
					t = ret;
					body = b;
				}
	}
}
