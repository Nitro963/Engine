#include "RigidBody.h"

#include <iostream>

#pragma region staticVar
std::vector<float> SolidSphere::vertices;
std::vector<unsigned int> SolidSphere::indices;

std::vector<float> SolidCuboid::vertices = {
-0.5f, -0.5f, -0.5f,
0.5f, -0.5f, -0.5f,
0.5f, 0.5f, -0.5f,
-0.5f, 0.5f, -0.5f,


-0.5f, -0.5f, 0.5f,
0.5f, -0.5f, 0.5f,
0.5f, 0.5f, 0.5f,
-0.5f, 0.5f, 0.5f,

-0.5f, 0.5f, 0.5f,
-0.5f, 0.5f, -0.5f,
-0.5f, -0.5f, -0.5f,
-0.5f, -0.5f, 0.5f,

0.5f, 0.5f, 0.5f,
0.5f, 0.5f, -0.5f,
0.5f, -0.5f, -0.5f,
0.5f, -0.5f, 0.5f,

-0.5f, -0.5f, -0.5f,
0.5f, -0.5f, -0.5f,
0.5f, -0.5f, 0.5f,
-0.5f, -0.5f, 0.5f,

-0.5f, 0.5f, -0.5f,
0.5f, 0.5f, -0.5f,
0.5f, 0.5f, 0.5f,
-0.5f, 0.5f, 0.5f,
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

RigidBody::RigidBody(const float mass, const glm::mat3  tensorBody, const glm::vec3&  position, const glm::fquat& orientation, const glm::vec3&  velocity, const glm::vec3& omega) : invMass(1.f / mass), tensorBody(tensorBody), invTensorBody(glm::inverse(tensorBody)), forceAccum(), torqueAccum(), awake(0), alive(1) {
	this->position = position;
	this->orientation = orientation;
	this->velocity = velocity;
	this->omega = omega;
	//this->linearMomentum = velocity * mass;
	rotation = glm::toMat3(orientation);
	rotationT = glm::transpose(rotation);
	invTensor = rotation * invTensorBody * rotationT;
	//this->angularMomentum = glm::inverse(invTensor) * omega;
}

void RigidBody::integrate(float duration) {
	if (isDead())
		return;
	velocity += forceAccum * invMass * duration;
	velocity *= 0.99f;

	omega += invTensor * torqueAccum * duration;
	omega *= 0.99f;

	if (glm::dot(velocity, velocity) < 0.001 && glm::dot(omega, omega) < 0.001)
		velocity = glm::vec3(0), omega = glm::vec3(0), awake = 0;
	else
		awake = 1;

	position += velocity * duration;

	orientation += glm::fquat(0, omega) * orientation * duration * 0.5f;

	orientation = glm::normalize(orientation);

	calcDerivedQuantities();

	//linearMomentum += forceAccum * duration;

	//angularMomentum += torqueAccum * duration;

	syncColliders();

	clearAccum();
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
	rotationT = glm::transpose(rotation);
	invTensor = rotation * invTensorBody * rotationT;
	//omega = invTensor * angularMomentum * 0.8f;
}

void RigidBody::applyForce(const glm::vec3& point, const glm::vec3& force) {
	forceAccum += force;

	torqueAccum += glm::cross(point, force);
}

void RigidBody::applyForce(const glm::vec3 & force) {
	forceAccum += force;
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
		double z0 = sin(lat0);
		double zr0 = cos(lat0);

		double lat1 = glm::pi<float>() * (-0.5 + (float)i / n);
		double z1 = sin(lat1);
		double zr1 = cos(lat1);

		for (int j = 0; j <= m; j++) {
			double lng = 2 * glm::pi<float>() * (float)(j - 1) / m;
			double x = cos(lng);
			double y = sin(lng);

			vertices.push_back(x * zr0);
			vertices.push_back(y * zr0);
			vertices.push_back(z0);
			indices.push_back(indicator++);

			vertices.push_back(x * zr1);
			vertices.push_back(y * zr1);
			vertices.push_back(z1);
			indices.push_back(indicator++);
		}
		indices.push_back(GL_PRIMITIVE_RESTART_FIXED_INDEX);
	}
}

SolidSphere::SolidSphere(const float mass, const float radius, const glm::vec3  position, const glm::fquat  orientation, const glm::vec3 velocity, const glm::vec3  omega) : RigidBody(mass, generateTensor(mass, radius), position, orientation, velocity, omega), radius(radius) {

	VBO = std::make_unique<renderer::vertexbuffer>(&vertices[0], vertices.size() * sizeof(float));
	renderer::vertexbufferlayout layout;
	layout.push<float>(3);
	VAO = std::make_unique<renderer::vertexarray>();
	VAO->addbuffer(*VBO, layout);
	IBO = std::make_unique<renderer::indexbuffer>(&indices[0], indices.size());

	collider1 = std::make_unique<BoundingSphere>(position, radius);
}

void SolidSphere::applyForce(const glm::vec3& point, const glm::vec3& force) {
	if (glm::dot(point, point) - radius * radius > EPSILON)
		return;
	RigidBody::applyForce(point, force);
}

void SolidSphere::render() const {
	GLCall(glEnable(GL_PRIMITIVE_RESTART));
	glPrimitiveRestartIndex(GL_PRIMITIVE_RESTART_FIXED_INDEX);
	VAO->bind();
	IBO->bind();
	GLCall(glDrawElements(GL_TRIANGLE_FAN, IBO->getcount(), GL_UNSIGNED_INT, NULL));
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

SolidCuboid::SolidCuboid(const float mass, const glm::vec3& extents, const glm::vec3 position, const glm::fquat orientation, const glm::vec3 velocity, const glm::vec3 omega) : RigidBody(mass, generateTensor(mass, extents), position, orientation, velocity, omega), extents(extents) {

	VBO = std::make_unique<renderer::vertexbuffer>(&vertices[0], vertices.size() * sizeof(float));
	renderer::vertexbufferlayout layout;
	layout.push<float>(3);
	VAO = std::make_unique<renderer::vertexarray>();
	VAO->addbuffer(*VBO, layout);
	IBO = std::make_unique<renderer::indexbuffer>(&indices[0], indices.size());

	collider2 = std::make_unique<OBB>(position, rotation, extents * 0.5f);
}

void SolidCuboid::applyForce(const glm::vec3 & point, const glm::vec3 & force) {
	glm::vec3 diff = glm::clamp(point, -extents * 0.5f, extents * 0.5f) - point;
	if (glm::dot(diff, diff) > EPSILON2)
		return;
	RigidBody::applyForce(point, force);
}

void SolidCuboid::render() const {
	VAO->bind();
	IBO->bind();
	GLCall(glDrawElements(GL_TRIANGLES, IBO->getcount(), GL_UNSIGNED_INT, NULL));
	VAO->unbind();
	IBO->unbind();
}

void applyImpulse(ContactData* contact, float epsilon) {

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

		float numerator = -(1.f + epsilon) * vrel;

		glm::vec3 term1 = glm::cross(contact->A->invTensor * glm::cross(ra, contact->M->normal), ra);
		glm::vec3 term2 = glm::cross(contact->B->invTensor * glm::cross(rb, contact->M->normal), rb);
		float denominator = invMassSum + glm::dot(contact->M->normal, term1 + term2);

		float j = (numerator / denominator);
		if (j < EPSILON)
			continue;
		j /= contact->M->contacts.size();

		glm::vec3 impulse = j * contact->M->normal;
		glm::vec3 torqueA = glm::cross(ra, impulse);
		glm::vec3 torqueB = glm::cross(rb, impulse);

		contact->A->velocity = contact->A->velocity - impulse *	contact->A->invMass;
		contact->B->velocity = contact->B->velocity + impulse *	contact->B->invMass;

		contact->A->omega -= contact->A->invTensor * torqueA;
		contact->B->omega += contact->B->invTensor * torqueB;

		glm::vec3 t = relativeVel - (contact->M->normal * glm::dot(relativeVel, contact->M->normal));
		if (glm::dot(t, t) < EPSILON2)
			continue;
		t = glm::normalize(t);

		numerator = -glm::dot(relativeVel, t);

		term1 = glm::cross(contact->A->invTensor * glm::cross(ra, t), ra);
		term2 = glm::cross(contact->B->invTensor * glm::cross(rb, t), rb);
		denominator = invMassSum + glm::dot(t, term1 + term2);

		float jt = numerator / denominator;
		if (glm::abs(jt) < EPSILON)
			continue;
		jt /= contact->M->contacts.size();

		float friction = glm::sqrt(0.3 * 0.01);

		jt = glm::clamp(jt, -j * friction, j * friction);

		glm::vec3 tangentImpuse = t * jt;
		torqueA = glm::cross(ra, tangentImpuse);
		torqueB = glm::cross(rb, tangentImpuse);

		contact->A->velocity = contact->A->velocity - tangentImpuse *	contact->A->invMass;
		contact->B->velocity = contact->B->velocity + tangentImpuse *	contact->B->invMass;

		contact->A->omega -= contact->A->invTensor * torqueA;
		contact->B->omega += contact->B->invTensor * torqueB;
	}
}

void resolveInterpentration(ContactData* contact) {

	float totalMass = contact->A->invMass + contact->B->invMass;

	if (totalMass < EPSILON)
		return;

	float depth = glm::max(contact->M->depth - 0.01f, 0.0f);
	float scalar = depth / totalMass;
	glm::vec3 correction = contact->M->normal * scalar * 0.45f;

	contact->A->position -= correction * contact->A->invMass;
	contact->B->position += correction * contact->B->invMass;

	contact->A->syncColliders();
	contact->B->syncColliders();
}
