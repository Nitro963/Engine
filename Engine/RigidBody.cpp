#include "RigidBody.h"

#include <iostream>

#pragma region staticVar
std::vector<float> SolidSphere::vertices;
std::vector<uint32_t> SolidSphere::indices;
std::unique_ptr<renderer::vertexbuffer>  SolidSphere::VBO = nullptr;
std::unique_ptr<renderer::vertexarray> SolidSphere::VAO = nullptr;
std::unique_ptr<renderer::indexbuffer> SolidSphere::IBO = nullptr;

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
std::vector<uint32_t> SolidCuboid::indices = {
	0 ,1 ,2,
	2 ,3 ,0,

	4 ,5 ,6,
	6 ,7 ,4,

	8 ,9 ,10,
	10 ,11 ,8,

	12 ,13 ,14,
	14 ,15 ,12,

	16 ,17 ,18,
	18 ,19 ,16,

	20 ,21 ,22,
	22 ,23 ,20,
};
std::unique_ptr<renderer::vertexbuffer>  SolidCuboid::VBO = nullptr;
std::unique_ptr<renderer::vertexarray> SolidCuboid::VAO = nullptr;
std::unique_ptr<renderer::indexbuffer> SolidCuboid::IBO = nullptr;

#pragma endregion staticVar

void addTriangle(std::vector<uint32_t>& indices, uint32_t a, uint32_t b, uint32_t c) {
	indices.push_back(a);
	indices.push_back(b);
	indices.push_back(c);
}

void addQuad(std::vector<uint32_t>& indices, uint32_t a, uint32_t b, uint32_t c, uint32_t d) {
		indices.push_back(a);
		indices.push_back(b);
		indices.push_back(c);
		indices.push_back(a);
		indices.push_back(c);
		indices.push_back(d);
}

RigidBody::RigidBody(const float mass, const glm::mat3  tensorBody, const glm::vec3&  position, const glm::fquat& orientation, const glm::vec3&  velocity, const glm::vec3& omega) : invMass(1.f / mass), tensorBody(tensorBody), invTensorBody(glm::inverse(tensorBody)), forceAccum(), torqueAccum() ,colliderType(0) {
	this->position = position;
	this->orientation = orientation;
	this->velocity = velocity;
	this->omega = omega;
	this->linearMomentum = velocity * mass;
	rotation = glm::toMat3(orientation);
	rotationT = glm::transpose(rotation);
	invTensor = rotation * invTensorBody * rotationT;

	this->angularMomentum = glm::inverse(invTensor) * omega;
}

void RigidBody::integrate(float duration) {
	if (glm::dot(velocity, velocity) < EPSILON)
		velocity = glm::vec3(0);
	
	if (glm::dot(omega, omega) < EPSILON)
		omega = glm::vec3(0);

	position += velocity * duration;

	orientation += glm::fquat(0 ,omega) * orientation * duration * 0.5f;
	
	orientation = glm::normalize(orientation);

	linearMomentum += forceAccum * duration;

	angularMomentum += torqueAccum * duration;

	calcDerivedQuantities();

	syncColliders();

	clearAccum();
}

void RigidBody::syncColliders(){
	if (colliderType == 1)
		collider1->sync(position);
	if (colliderType == 2)
		collider2->sync(position, rotationT);
}

void RigidBody::calcDerivedQuantities() {

	velocity = linearMomentum * invMass * 0.8f;
	rotation = glm::toMat3(orientation);
	rotationT = glm::transpose(rotation);
	invTensor = rotation * invTensorBody * rotationT;
	omega = invTensor * angularMomentum * 0.8f;
}

void RigidBody::applyForce(const glm::vec3& point, const glm::vec3& force) {
	forceAccum += force;

	torqueAccum += glm::cross(point, force);
}

void RigidBody::applyForce(const glm::vec3 & force){
	forceAccum += force;
}

point RigidBody::transformWorld(const point& pt) const{
	return pt - position;
}

point RigidBody::transformLocal(const point & pt) const {
	return rotation * pt + position;
}

inline glm::mat3 SolidSphere::generateTensor(float mass, float radius) {
	glm::mat3 tensor(0.f);
	tensor[0][0] = tensor[1][1] = tensor[2][2] = mass * radius * radius * 2 / 5;
	return tensor;
}

void SolidSphere::generateVertices(){
	int indicator = 0;
	const uint32_t n = 40;
	const uint32_t m = 40;
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

SolidSphere::SolidSphere(const float mass, const float radius, const glm::vec3&  position, const glm::fquat&  orientation, const glm::vec3&  velocity, const glm::vec3&  omega) : RigidBody(mass, generateTensor(mass, radius), position, orientation, velocity, omega), radius(radius) { 
	if (!VBO) {
		generateVertices();
		VBO = std::make_unique<renderer::vertexbuffer>(&vertices[0], vertices.size() * sizeof(float));
		renderer::vertexbufferlayout layout;
		layout.push<float>(3);
		VAO = std::make_unique<renderer::vertexarray>();
		VAO->addbuffer(*VBO, layout);
		IBO = std::make_unique<renderer::indexbuffer>(&indices[0], indices.size());
	}
	colliderType = 1;
	collider1 = std::make_unique<boundingSphere>(position, radius);
}

void SolidSphere::applyForce(const glm::vec3& point, const glm::vec3& force) {
	if (glm::dot(point, point) - radius * radius > EPSILON)
		return;
	RigidBody::applyForce(point, force);
}

void SolidSphere::render(){
	GLCall(glEnable(GL_PRIMITIVE_RESTART));
	glPrimitiveRestartIndex(GL_PRIMITIVE_RESTART_FIXED_INDEX);
	VAO->bind();
	IBO->bind();
	GLCall(glDrawElements(GL_TRIANGLE_FAN, IBO->getcount(), GL_UNSIGNED_INT, NULL));
	VAO->unbind();
	IBO->unbind();
	GLCall(glDisable(GL_PRIMITIVE_RESTART));
}

inline glm::mat3 SolidCuboid::generateTensor(float mass, const glm::vec3 & extents){
	glm::mat3 ret(0.f);
	ret[0][0] = mass * (extents.y * extents.y + extents.z * extents.z) / 12;
	ret[1][1] = mass * (extents.x * extents.x + extents.z * extents.z) / 12;
	ret[2][2] = mass * (extents.x * extents.x + extents.y * extents.y) / 12;
	return ret;
}

SolidCuboid::SolidCuboid(const float mass, const glm::vec3& extents, const glm::vec3 & position, const glm::fquat & orientation, const glm::vec3 & velocity, const glm::vec3 & omega) : RigidBody(mass, generateTensor(mass, extents), position, orientation, velocity, omega), extents(extents) {
	if (!VBO) {
		VBO = std::make_unique<renderer::vertexbuffer>(&vertices[0], vertices.size() * sizeof(float));
		renderer::vertexbufferlayout layout;
		layout.push<float>(3);
		VAO = std::make_unique<renderer::vertexarray>();
		VAO->addbuffer(*VBO, layout);
		IBO = std::make_unique<renderer::indexbuffer>(&indices[0], indices.size());
	}
	colliderType = 2;
	collider2 = std::make_unique<OBB>(position, rotationT, extents * 0.5f);
}

void SolidCuboid::applyForce(const glm::vec3 & point, const glm::vec3 & force){
	glm::vec3 diff = glm::clamp(point, -extents * 0.5f, extents * 0.5f) - point;
	if (glm::dot(diff ,diff) > EPSILON2)
		return;
	RigidBody::applyForce(point, force);
}

void SolidCuboid::render(){
	VAO->bind();
	IBO->bind();
	GLCall(glDrawElements(GL_TRIANGLES, IBO->getcount(), GL_UNSIGNED_INT, NULL));
	VAO->unbind();
	IBO->unbind();
}

void applyImpulse(Contact* contact, float epsilon) {
	for (const auto& p : contact->M->contacts) {

		float invMassSum = contact->A->invMass + contact->B->invMass;

		if (invMassSum < EPSILON)
			return; // Both objects have infinite mass!

		glm::vec3 ra = contact->A->transformWorld(p);
		glm::vec3 rb = contact->B->transformWorld(p);

		glm::vec3 relativeVel = contact->B->getParticleVelocity(p) - contact->A->getParticleVelocity(p);
		float vrel = glm::dot(relativeVel, contact->M->normal);
		// if the objects are Moving away from each other at p we will skip it 
		if (vrel > 0.f)
			continue;
		//if (glm::abs(vrel < EPSILON))
			//continue;
		float numerator = -(1.f + epsilon) * vrel;

		glm::vec3 term1 = glm::cross(contact->A->invTensor * glm::cross(ra, contact->M->normal), ra);
		glm::vec3 term2 = glm::cross(contact->B->invTensor * glm::cross(rb, contact->M->normal), rb);
		float denominator = invMassSum + glm::dot(contact->M->normal, term1 + term2);

		if (glm::abs(denominator) < EPSILON)
			continue;

		float j = numerator / denominator;
		
		//if (glm::abs(j) > EPSILON)
		//	j = j / M.contacts.size();

		glm::vec3 impulse = j * contact->M->normal;
		glm::vec3 torqueA = glm::cross(ra, impulse);
		glm::vec3 torqueB = glm::cross(rb, impulse);

		//update linear velocity
		contact->A->velocity -= impulse *  contact->A->invMass;
		contact->B->velocity += impulse *  contact->B->invMass;
		//update angular velocity
		contact->A->omega -= contact->A->invTensor * glm::cross(ra, impulse);
		contact->B->omega += contact->B->invTensor * glm::cross(rb, impulse);

		//update momentum
		contact->A->linearMomentum -= impulse;
		contact->A->angularMomentum -= torqueA;
		contact->B->linearMomentum += impulse;
		contact->B->angularMomentum += torqueB;
	}
}

void resolveInterpentration(Contact* contact){
	float depth = glm::max(contact->M->depth , 0.f);
	float scaler = depth / (contact->A->invMass + contact->B->invMass);
	glm::vec3 correction = scaler * contact->M->normal * 0.45f;
	contact->A->position -= correction * contact->A->invMass;
	contact->B->position += correction * contact->B->invMass;
	
	contact->A->syncColliders();
	contact->B->syncColliders();
}

std::vector<Contact*> searchForContacts(std::vector<RigidBody*>& bodies){
	std::vector<Contact*> ve;
	for(int i = 0 ; i < bodies.size();i++)
		for (int j = i + 1; j < bodies.size(); j++) {
			RigidBody* A = bodies[i];
			RigidBody* B = bodies[j];
			int typeA = A->colliderType;
			int typeB = B->colliderType;
			CollisionManifold M;
			bool flip = 0;
			if (typeA == 1)
				if (typeB == 1)
					M = A->collider1->findCollisionFeatures(*B->collider1);
				else
					M = A->collider1->findCollisionFeatures(*B->collider2);
			else
				if (typeB == 1)
					M = B->collider1->findCollisionFeatures(*A->collider2), flip = 1;
				else
					M = A->collider2->findCollisionFeatures(*B->collider2);
			if (M.colliding)
				if (!flip)
					ve.push_back(new Contact(A, B, new CollisionManifold(M)));	
				else
					ve.push_back(new Contact(B, A, new CollisionManifold(M)));
				
		}
	return ve;
}
