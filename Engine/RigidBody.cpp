#include "RigidBody.h"

#include <iostream>

#pragma region staticVar
float RigidBody::damping = 0.99;
std::vector<float> SolidSphere::vertices;
std::vector<unsigned int> SolidSphere::indices;

std::vector<float> SolidCuboid::vertices = {
-0.5f, -0.5f, -0.5f,   0.0f,  0.0f, -1.0f, .2f, 0.7f, 0.6f,
0.5f, -0.5f, -0.5f,	  0.0f,  0.0f, -1.0f, 0.2f, 0.7f, 0.6f,
0.5f, 0.5f, -0.5f,	  0.0f,  0.0f, -1.0f, 0.2f, 0.7f, 0.6f,
-0.5f, 0.5f, -0.5f,	   0.0f,  0.0f, -1.0f, .2f, 0.7f, 0.6f,

-0.5f, -0.5f, 0.5f, 0.0f,  0.0f, 1.0f, 0.2f, 0.5f, 0.6f,
0.5f, -0.5f, 0.5f, 0.0f,  0.0f, 1.0f, 0.2f, 0.5f, 0.6f,
0.5f, 0.5f, 0.5f, 0.0f,  0.0f, 1.0f, 0.2f, 0.5f, 0.6f,
-0.5f, 0.5f, 0.5f, 0.0f,  0.0f, 1.0f, 0.2f, 0.5f, 0.6f,

-0.5f, 0.5f, 0.5f, -1.0f,  0.0f,  0.0f, 0.2f, 0.5f, 0.6f,
-0.5f, 0.5f, -0.5f, -1.0f,  0.0f,  0.0f, 0.2f, 0.5f, 0.6f,
-0.5f, -0.5f, -0.5f, -1.0f,  0.0f,  0.0f, 0.2f, 0.5f, 0.6f,
-0.5f, -0.5f, 0.5f, -1.0f,  0.0f,  0.0f, 0.2f, 0.5f, 0.6f,

0.5f, 0.5f, 0.5f, 1.0f,  0.0f,  0.0f, 0.2f, 0.7f, 0.6f,
0.5f, 0.5f, -0.5f, 1.0f,  0.0f,  0.0f, 0.2f, 0.7f, 0.6f,
0.5f, -0.5f, -0.5f, 1.0f,  0.0f,  0.0f, 0.2f, 0.7f, 0.6f,
0.5f, -0.5f, 0.5f, 1.0f,  0.0f,  0.0f, 0.2f, 0.7f, 0.6f,

-0.5f, -0.5f, -0.5f, 0.0f, -1.0f,  0.0f, 0.2f, 0.7f, 0.6f,
0.5f, -0.5f, -0.5f, 0.0f, -1.0f,  0.0f, 0.2f, 0.7f, 0.6f,
0.5f, -0.5f, 0.5f, 0.0f, -1.0f,  0.0f, 0.2f, 0.7f, 0.6f,
-0.5f, -0.5f, 0.5f, 0.0f, -1.0f,  0.0f, 0.2f, 0.7f, 0.6f,

-0.5f, 0.5f, -0.5f, 0.0f,  1.0f,  0.0f,0.2f, 0.5f, 0.6f,
0.5f, 0.5f, -0.5f, 0.0f,  1.0f,  0.0f, 0.2f, 0.5f, 0.6f,
0.5f, 0.5f, 0.5f, 0.0f,  1.0f,  0.0f,  0.2f, 0.5f, 0.6f,
-0.5f, 0.5f, 0.5f, 0.0f,  1.0f,  0.0f, 0.2f, 0.5f, 0.6f,
};
std::vector<unsigned int> SolidCuboid::indices = {
	0, 1, 2,
	2, 3, 0,

	4, 5, 6,
	6, 7, 4,

	8, 9, 10,
	10, 11, 8,

	12, 13, 14,
	14, 15, 12,

	16, 17, 18,
	18, 19, 16,

	20, 21, 22,
	22, 23, 20,
};
#pragma endregion staticVar

void addTriangle(std::vector<unsigned int>& indices, unsigned int a, unsigned int b, unsigned int c) {
	indices.push_back(a);
	indices.push_back(b);
	indices.push_back(c);
}

void addQuad(std::vector<unsigned int>& indices, unsigned int a, unsigned int b, unsigned int c, unsigned int d) {
	indices.push_back(a);
	indices.push_back(b);
	indices.push_back(c);
	indices.push_back(a);
	indices.push_back(c);
	indices.push_back(d);
}

RigidBody::RigidBody(const float mass, const glm::mat3 tensorBody, const Material& bodyMaterial, const glm::vec3&  position, const glm::fquat& orientation, const glm::vec3&  velocity, const glm::vec3& omega) : invMass(1.f / mass), tensorBody(tensorBody), invTensorBody(glm::inverse(tensorBody)), bodyMaterial(bodyMaterial), alive(1), position(position), orientation(orientation), velocity(velocity), omega(omega) {
	rotation = glm::toMat3(orientation);
	invTensor = rotation * invTensorBody * glm::transpose(rotation);

	if (glm::dot(velocity, velocity) < VELOCITYLIMIT && glm::dot(omega, omega) < VELOCITYLIMIT)
		this->velocity = glm::vec3(0), this->omega = glm::vec3(0), awake = 0;
	else
		awake = 1;

}

void RigidBody::integrate(float duration) {
	if (isDead())
		return;

	velocity *= damping;
	omega *= damping;

	if (glm::dot(velocity, velocity) < VELOCITYLIMIT && glm::dot(omega, omega) < VELOCITYLIMIT)
		velocity = glm::vec3(0), omega = glm::vec3(0), awake = 0;
	else
		awake = 1;

	position += velocity * duration;

	orientation += glm::fquat(0, omega) * orientation * duration * 0.5f;

	orientation = glm::normalize(orientation);

	calcDerivedQuantities();

	syncColliders();
}

void RigidBody::syncColliders() {
	if (collider1)
		collider1->sync(position);
	if (collider2)
		collider2->sync(position, rotation);
}

void RigidBody::calcDerivedQuantities() {

	//velocity = linearMomentum * invMass * 0.8f;
	rotation = glm::toMat3(orientation);
	invTensor = rotation * invTensorBody * glm::transpose(rotation);
	//omega = invTensor * angularMomentum * 0.8f;
}

void RigidBody::applyImpulse(const glm::vec3& impulse, const glm::vec3& impulsiveTorque) {
	velocity += impulse * invMass;

	omega += invTensor * impulsiveTorque;
}

void RigidBody::applyImpulse(const glm::vec3 & impulse) {
	velocity += impulse * invMass;
}

point RigidBody::transform(const point& pt) const {
	return pt - position;
}

point RigidBody::transformInverse(const point & pt) const {
	return rotation * pt + position;
}

inline glm::mat3 SolidSphere::generateTensor(float mass, float radius) {
	glm::mat3 tensor(0.f);
	tensor[0][0] = tensor[1][1] = tensor[2][2] = mass * radius * radius * 2 / 5;
	return tensor;
}

void SolidSphere::generateVertices() {
	indices.clear();
	vertices.clear();
	int indicator = 0;
	const unsigned int n = 40;
	const unsigned int m = 40;
	for (int i = 0; i <= n; i++) {
		double lat0 = glm::pi<float>() * (-0.5 + (float)(i - 1) / n);
		double z0 = glm::sin(lat0);
		double zr0 = glm::cos(lat0);

		double lat1 = glm::pi<float>() * (-0.5 + (float)i / n);
		double z1 = glm::sin(lat1);
		double zr1 = glm::cos(lat1);

		for (int j = 0; j <= m; j++) {
			double lng = 2 * glm::pi<float>() * (float)(j - 1) / m;
			double x = glm::cos(lng);
			double y = glm::sin(lng);

			glm::vec3 v(x * zr0, y * zr0, z0);
			glm::vec3 vn = glm::normalize(v);
			glm::vec3 u(x * zr1, y * zr1, z1);
			glm::vec3 un = glm::normalize(u);

			vertices.push_back(v.x);
			vertices.push_back(v.y);
			vertices.push_back(v.z);
			vertices.push_back(vn.x);
			vertices.push_back(vn.y);
			vertices.push_back(vn.z);

			vertices.push_back(0.f);
			vertices.push_back(i < 20 ? 1.f : 0.5);
			vertices.push_back(0.4);

			indices.push_back(indicator++);

			vertices.push_back(u.x);
			vertices.push_back(u.y);
			vertices.push_back(u.z);
			vertices.push_back(un.x);
			vertices.push_back(un.y);
			vertices.push_back(un.z);

			vertices.push_back(0.f);
			vertices.push_back(i < 20 ? 1.f : 0.5);
			vertices.push_back(0.4);

			indices.push_back(indicator++);
		}
		indices.push_back(GL_PRIMITIVE_RESTART_FIXED_INDEX);
	}
}

SolidSphere::SolidSphere(const float mass, const float radius, const Material& bodyMaterial, const glm::vec3  position, const glm::fquat  orientation, const glm::vec3 velocity, const glm::vec3  omega) : RigidBody(mass, generateTensor(mass, radius), bodyMaterial, position, orientation, velocity, omega), radius(radius) {

	VBO = std::make_unique<renderer::vertexbuffer>(&vertices[0], vertices.size() * sizeof(float));
	renderer::vertexbufferlayout layout;
	layout.push<float>(3);
	layout.push<float>(3);
	layout.push<float>(3);
	VAO = std::make_unique<renderer::vertexarray>();
	VAO->addbuffer(*VBO, layout);
	IBO = std::make_unique<renderer::indexbuffer>(&indices[0], indices.size());

	collider1 = std::make_unique<BoundingSphere>(position, radius);
}

void SolidSphere::render() const {
	GLCall(glEnable(GL_PRIMITIVE_RESTART));
	glPrimitiveRestartIndex(GL_PRIMITIVE_RESTART_FIXED_INDEX);
	VAO->bind();
	IBO->bind();
	GLCall(glDrawElements(GL_TRIANGLE_STRIP, IBO->getcount(), GL_UNSIGNED_INT, NULL));
	VAO->unbind();
	IBO->unbind();
	GLCall(glDisable(GL_PRIMITIVE_RESTART));
}

inline glm::mat3 SolidCuboid::generateTensor(float mass, const glm::vec3 & extents) {
	glm::mat3 ret(0.f);
	ret[0][0] = mass * (extents.y * extents.y + extents.z * extents.z) / 12;
	ret[1][1] = mass * (extents.x * extents.x + extents.z * extents.z) / 12;
	ret[2][2] = mass * (extents.x * extents.x + extents.y * extents.y) / 12;
	return ret;
}

SolidCuboid::SolidCuboid(const float mass, const glm::vec3& extents, const Material& bodyMaterial, const glm::vec3 position, const glm::fquat orientation, const glm::vec3 velocity, const glm::vec3 omega) : RigidBody(mass, generateTensor(mass, extents), bodyMaterial, position, orientation, velocity, omega), extents(extents) {

	VBO = std::make_unique<renderer::vertexbuffer>(&vertices[0], vertices.size() * sizeof(float));
	renderer::vertexbufferlayout layout;
	layout.push<float>(3);
	layout.push<float>(3);
	layout.push<float>(3);
	VAO = std::make_unique<renderer::vertexarray>();
	VAO->addbuffer(*VBO, layout);
	IBO = std::make_unique<renderer::indexbuffer>(&indices[0], indices.size());

	collider2 = std::make_unique<OBB>(position, rotation, extents * 0.5f);
}

void SolidCuboid::render() const {
	VAO->bind();
	IBO->bind();
	GLCall(glDrawElements(GL_TRIANGLES, IBO->getcount(), GL_UNSIGNED_INT, NULL));
	VAO->unbind();
	IBO->unbind();
}

void resolveContact(ContactData* contact) {
	float invMassSum = contact->B->invMass + contact->A->invMass;

	if (invMassSum < EPSILON)
		return; // Both objects have infinite mass!

	for (const auto& p : contact->M->contacts) {

		glm::vec3 ra = contact->A->transform(p);
		glm::vec3 rb = contact->B->transform(p);

		glm::vec3 relativeVel = contact->B->getParticleVelocity(p) - contact->A->getParticleVelocity(p);
		float vrel = glm::dot(relativeVel, contact->M->normal);
		// if the objects are Moving away from each other at p we will skip it 
		if (vrel > 0.f)
			continue;
		float epsilon = glm::min(contact->A->bodyMaterial.epsilon, contact->B->bodyMaterial.epsilon);
		float numerator = -(1.f + epsilon) * vrel;

		glm::vec3 term1 = glm::cross(contact->A->invTensor * glm::cross(ra, contact->M->normal), ra);
		glm::vec3 term2 = glm::cross(contact->B->invTensor * glm::cross(rb, contact->M->normal), rb);
		float denominator = invMassSum + glm::dot(contact->M->normal, term1 + term2);
		if (glm::abs(denominator) < EPSILON)
			denominator += EPSILON;
		float j = (numerator / denominator);
		if (glm::abs(j) < EPSILON)
			continue;
		j /= contact->M->contacts.size();

		glm::vec3 impulse = j * contact->M->normal;

		glm::vec3 torqueA = glm::cross(ra, impulse);
		glm::vec3 torqueB = glm::cross(rb, impulse);

		contact->A->velocity -= impulse * contact->A->invMass;
		contact->B->velocity += impulse * contact->B->invMass;

		contact->A->omega -= contact->A->invTensor * torqueA;
		contact->B->omega += contact->B->invTensor * torqueB;

		glm::vec3 t = relativeVel - (contact->M->normal * vrel);
		if (glm::dot(t, t) < EPSILON2)
			continue;
		t = glm::normalize(t);

		numerator = -(1 + epsilon) * glm::dot(relativeVel, t);

		term1 = glm::cross(contact->A->invTensor * glm::cross(ra, t), ra);
		term2 = glm::cross(contact->B->invTensor * glm::cross(rb, t), rb);
		denominator = invMassSum + glm::dot(t, term1 + term2);
		if (glm::abs(denominator) < EPSILON)
			denominator += EPSILON;

		float jt = numerator / denominator;
		if (glm::abs(jt) < EPSILON)
			continue;
		jt /= contact->M->contacts.size();

		float friction = glm::sqrt(contact->A->bodyMaterial.mu * contact->B->bodyMaterial.mu);

		jt = glm::clamp(jt, -j * friction, j * friction);

		glm::vec3 tangentImpuse = t * jt;

		if (vrel < -0.01) {
			//dynamic friction
			float df = sqrtf(contact->A->bodyMaterial.muDynamic * contact->B->bodyMaterial.muDynamic);
			if (glm::abs(jt) > j * friction)
				tangentImpuse = -j * t * df;//lenght of normal force * tangent * Coefficient of dynamic friction
		}

		torqueA = glm::cross(ra, tangentImpuse);
		torqueB = glm::cross(rb, tangentImpuse);

		contact->A->velocity -= tangentImpuse *	contact->A->invMass;
		contact->B->velocity += tangentImpuse *	contact->B->invMass;

		contact->A->omega -= contact->A->invTensor * torqueA;
		contact->B->omega += contact->B->invTensor * torqueB;

		if (glm::dot(contact->A->velocity, contact->A->velocity) > 0.01 || glm::dot(contact->A->omega, contact->A->omega) > 0.01)
			contact->A->awake = 1;
		if (glm::dot(contact->B->velocity, contact->B->velocity) > 0.01 || glm::dot(contact->B->omega, contact->B->omega) > 0.01)
			contact->B->awake = 1;
	}
}

void resolveInterpentration(ContactData* contact) {

	float totalMass = contact->A->invMass + contact->B->invMass;

	if (totalMass < EPSILON)
		return;

	float depth = glm::max(contact->M->depth - 0.01f, 0.0f);
	float scalar = depth / totalMass;
	glm::vec3 correction = contact->M->normal * scalar * 0.6f;

	contact->A->position -= correction * contact->A->invMass;
	contact->B->position += correction * contact->B->invMass;

	contact->A->syncColliders();
	contact->B->syncColliders();
}
