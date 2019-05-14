#include "RigidBody.h"

#include <iostream>

#pragma region staticVar
std::vector<float> SolidSphere::vertices;
std::vector<uint32_t> SolidSphere::indices;
renderer::vertexbuffer* SolidSphere::VBO = nullptr;
renderer::vertexarray* SolidSphere::VAO = nullptr;
renderer::indexbuffer* SolidSphere::IBO = nullptr;

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
renderer::vertexbuffer* SolidCuboid::VBO = nullptr;
renderer::vertexarray* SolidCuboid::VAO = nullptr;
renderer::indexbuffer* SolidCuboid::IBO = nullptr;

Floor* Floor::object = nullptr;
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

RigidBody::RigidBody(const float mass, const glm::mat3  tensorBody, const glm::vec3&  position, const glm::fquat& orientation, const glm::vec3&  velocity, const glm::vec3& omega) : invMass(1.f / mass), tensorBody(tensorBody), invTensorBody(glm::inverse(tensorBody)), forceAccum(), torqueAccum() {
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
	position += velocity * duration;

	orientation += glm::fquat(0 ,omega) * orientation * duration * 0.5f;
	
	orientation = glm::normalize(orientation);

	linearMomentum += forceAccum * duration;

	angularMomentum += torqueAccum * duration;

	calcDerivedQuantities();

	clearAccum();
}

void RigidBody::calcDerivedQuantities() {

	velocity = linearMomentum * invMass;
	rotation = glm::toMat3(orientation);
	rotationT = glm::transpose(rotation);
	invTensor = rotation * invTensorBody * rotationT;
	omega = invTensor * angularMomentum;
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
	tensor[0][0] = tensor[1][1] = tensor[2][2] = mass * radius * 2 / 5;
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
		VBO = new renderer::vertexbuffer(&vertices[0], vertices.size() * sizeof(float));
		renderer::vertexbufferlayout layout;
		layout.push<float>(3);
		VAO = new renderer::vertexarray();
		VAO->addbuffer(*VBO, layout);
		IBO = new renderer::indexbuffer(&indices[0], indices.size());
	}
}

void SolidSphere::applyForce(const glm::vec3& point, const glm::vec3& force) {
	if (glm::dot(point, point) - radius * radius > EPSILON)
		return;
	RigidBody::applyForce(point, force);
}

bool SolidSphere::isColliding(SolidSphere* other, Contact* contactData){
	boundingSphere BV1(position, radius);
	boundingSphere BV2(other->position, other->radius);
	CollisionData data;
	if (BV1.testSphere(BV2, data)) {
		contactData->a = this;
		contactData->b = other;
		contactData->normal = data.contactNormal;
		contactData->point = data.collisionPoint;
		//TODO 
		//resolve interpentration
		return true;
	}
	return false;
}

bool SolidSphere::isColliding(SolidCuboid * other, Contact * contactData){
	boundingSphere BV1(position, radius);
	OBB BV2(other->getPosition(), other->getRotationT(), other->getExtents() * 0.5f);
	CollisionData data;
	if(BV1.testOBB(BV2 ,data)){
		contactData->a = other;
		contactData->b = this;
		contactData->normal = data.contactNormal;
		contactData->point = data.collisionPoint;
		return true;
	}
	return false;
}

void SolidSphere::draw(){
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
		VBO = new renderer::vertexbuffer(&vertices[0], vertices.size() * sizeof(float));
		renderer::vertexbufferlayout layout;
		layout.push<float>(3);
		VAO = new renderer::vertexarray();
		VAO->addbuffer(*VBO, layout);
		IBO = new renderer::indexbuffer(&indices[0], indices.size());
	}
}

void SolidCuboid::applyForce(const glm::vec3 & point, const glm::vec3 & force){
	glm::vec3 diff = glm::clamp(point, -extents * 0.5f, extents * 0.5f) - point;
	if (glm::dot(diff ,diff) > EPSILON2)
		return;
	RigidBody::applyForce(point, force);
}

void SolidCuboid::draw(){
	VAO->bind();
	IBO->bind();
	GLCall(glDrawElements(GL_TRIANGLES, IBO->getcount(), GL_UNSIGNED_INT, NULL));
	VAO->unbind();
	IBO->unbind();
}

bool isColliding(Contact * c){
	glm::vec3 velA = c->a->getParticleVelocity(c->point);
	glm::vec3 velB = c->b->getParticleVelocity(c->point);
	float vrel = glm::dot(c->normal, velA - velB);
	if (vrel < EPSILON)
		return true;
	return false;
}

void handelCollision(Contact * c, float epsilon){
	glm::vec3 velA = c->a->getParticleVelocity(c->point);
	glm::vec3 velB = c->b->getParticleVelocity(c->point);
	float vrel = glm::dot(c->normal, velA - velB);
	if (vrel > EPSILON)
		return;
	glm::vec3 ra = c->point - c->a->getPosition();
	glm::vec3 rb = c->point - c->b->getPosition();
	float numerator = -(1 + epsilon) * vrel;

	float term1 = c->a->getInverseMass();
	float term2 = c->b->getInverseMass();
	float term3 = glm::dot(c->normal, glm::cross(c->a->getInverseTensor() * glm::cross(ra, c->normal), ra));
	float term4 = glm::dot(c->normal, glm::cross(c->b->getInverseTensor() * glm::cross(rb, c->normal), rb));

	float j = numerator / (term1 + term2 + term3 + term4);

	glm::vec3 force = j * c->normal;
	glm::vec3 torqueA = glm::cross(c->a->transformWorld(c->point), force);
	glm::vec3 torqueB = glm::cross(c->b->transformWorld(c->point), -force);

	c->a->setLinearMomentum(force + c->a->getLinearMomentum());
	c->b->setLinearMomentum(-force + c->b->getLinearMomentum());
	c->a->setAngularMomentum(torqueA + c->a->getAngularMomentum());
	c->b->setAngularMomentum(torqueB + c->b->getAngularMomentum());
	glm::vec3 vel = c->a->getLinearMomentum() * c->a->getInverseMass();
	c->a->setVelocity(vel);
	vel = c->b->getLinearMomentum() * c->b->getInverseMass();
	c->b->setVelocity(vel);

	vel = c->a->getInverseTensor() * c->a->getAngularMomentum();
	c->a->setAngularVelocity(vel);
	vel = c->b->getInverseTensor() * c->b->getAngularMomentum();
	c->b->setAngularVelocity(vel);
}

Floor * Floor::getInstance(){
	if (!object)
		return object = new Floor();
	else
		return object;
}

bool Floor::isColliding(SolidSphere * other, Contact * contactData){
	boundingSphere BV(other->getPosition(), other->getRadius());
	if (collider.testSphere(BV))
		return true;
	return false;
}
