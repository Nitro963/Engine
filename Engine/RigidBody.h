#ifndef RIGID_BODY_H
#define RIGID_BODY_H
#include "glm\gtc\matrix_transform.hpp"
#include "glm\gtx\quaternion.hpp"
#include "glm\glm.hpp"
#include "glm\gtc\matrix_inverse.hpp"

#include <vector>
#include <memory>

#include "vertexbuffer.h"
#include "vertexarray.h"
#include "indexbuffer.h"
#include "Geometry.h"
#include "OBB.h"
#include "BoundingSphere.h"
#include "Plane.h"
#include "texture.h"

struct Contact;
class RigidBody {
public:

	RigidBody(const float mass, const glm::mat3 tensorBody, const glm::vec3& position = glm::vec3(), const glm::fquat& orientation = glm::angleAxis(glm::radians(0.f), glm::vec3(0, 0, 1)), const glm::vec3& velocity = glm::vec3(), const glm::vec3& omega = glm::vec3());

	void integrate(float duration);

	void syncColliders();

	//point is given in local space
	virtual void applyForce(const glm::vec3& point, const glm::vec3& force);

	void applyForce(const glm::vec3& force);

	point transformWorld(const point& pt) const;

	point transformLocal(const point& pt) const;

	inline const glm::vec3& getPosition() const { return position; }

	inline const glm::mat3& getRotation() const { return rotation; }

	inline const glm::mat3& getRotationT() const { return rotationT; }

	inline const glm::vec3& getAngularVelocity() const { return omega; }

	inline const glm::vec3& getVelocity() const { return velocity; }

	inline const glm::vec3& getAngularMomentum() const { return angularMomentum; }

	inline const glm::vec3& getLinearMomentum() const { return linearMomentum; }

	inline const glm::fquat& getOrientation() const { return orientation; }
	//point in world space
	const glm::vec3 getParticleVelocity(const glm::vec3& point) const {
		return velocity + glm::cross(omega, transformWorld(point));
	}
	
	inline const float getInverseMass() const{ return invMass; }

	inline const float getMass() const { return 1 / invMass; }

	inline const glm::mat3& getInverseTensor() const { return invTensor; }

	inline virtual const glm::mat4 getModel(glm::vec3 scaler) const { return glm::identity<glm::mat4>(); }
	
	virtual void render() {};
	

	friend void applyImpulse(Contact* contact, float epsilon);

	friend void resolveInterpentration(Contact* contact);

	friend std::vector<Contact*> searchForContacts(std::vector<RigidBody*>& bodies);

protected:
	glm::vec3 position;
	glm::fquat orientation;
	glm::vec3 linearMomentum;
	glm::vec3 angularMomentum;

	glm::mat3 invTensor;
	glm::mat3 rotation;
	glm::mat3 rotationT;
	glm::vec3 velocity;
	glm::vec3 omega;

	int colliderType;
	std::unique_ptr<boundingSphere> collider1;
	std::unique_ptr<OBB> collider2;

	//constant variable
	const glm::mat3 tensorBody;
	const glm::mat3 invTensorBody;
	const float invMass;

	glm::vec3 forceAccum;
	glm::vec3 torqueAccum;

	void calcDerivedQuantities();

	inline void clearAccum() { forceAccum[0] = forceAccum[1] = forceAccum[2] = torqueAccum[0] = torqueAccum[1] = torqueAccum[2] = 0; }
};

class SolidCuboid;
class SolidSphere : public RigidBody{
public:
	SolidSphere(const float mass, const float radius, const glm::vec3& position = glm::vec3(), const glm::fquat& orientation = glm::angleAxis(glm::radians(0.f) ,glm::vec3(0 ,0 ,1)), const glm::vec3& velocity = glm::vec3(), const glm::vec3& omega = glm::vec3());

	virtual void applyForce(const glm::vec3& point, const glm::vec3& force) override;
	
	inline float getRadius() { return radius; }
	
	inline virtual const glm::mat4 getModel(glm::vec3 scaler) const override {
		glm::mat4 model = glm::scale(glm::identity<glm::mat4>(), scaler);
		model = glm::translate(model, position) * glm::mat4(rotation);
		model = glm::scale(model, glm::vec3(radius));
		return model;
	};

	virtual void render() override;
private:
	const float radius;

	inline static glm::mat3 generateTensor(float mass, float radius);
	static void generateVertices();

	static std::vector<float> vertices;
	static std::vector<uint32_t> indices;
	static std::unique_ptr<renderer::vertexbuffer> VBO;
	static std::unique_ptr<renderer::vertexarray> VAO;
	static std::unique_ptr<renderer::indexbuffer> IBO;

};

class SolidCuboid : public RigidBody{
public:
	SolidCuboid(const float mass, const glm::vec3& extens, const glm::vec3& position = glm::vec3(), const glm::fquat& orientation = glm::angleAxis(glm::radians(0.f), glm::vec3(0, 0, 1)), const glm::vec3& velocity = glm::vec3(), const glm::vec3& omega = glm::vec3());

	virtual void applyForce(const glm::vec3& point, const glm::vec3& force) override;

	inline const glm::vec3& getExtents() { return extents; }
	
	inline virtual const glm::mat4 getModel(glm::vec3 scaler) const override {
		glm::mat4 model = glm::scale(glm::identity<glm::mat4>(), scaler);
		model = glm::translate(model, position) * glm::mat4(rotation);
		model = glm::scale(model, extents);
		return model;
	};

	virtual void render() override;
private:
	glm::vec3 extents;
	inline static glm::mat3 generateTensor(float mass, const glm::vec3& extents);

	static std::vector<float> vertices;
	static std::vector<uint32_t> indices;

	static std::unique_ptr<renderer::vertexbuffer> VBO;
	static std::unique_ptr<renderer::vertexarray> VAO;
	static std::unique_ptr<renderer::indexbuffer> IBO;
};

struct Contact {
	RigidBody* A;
	RigidBody* B;
	const CollisionManifold* M;

	Contact(RigidBody* A, RigidBody* B, const CollisionManifold* M) : A(A), B(B), M(M) {}
};

void applyImpulse(Contact* contact, float epsilon);

void resolveInterpentration(Contact* contact);

std::vector<Contact*> searchForContacts(std::vector<RigidBody*>& bodies);

#endif // !RIGID_BODY_H
