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
#include "Test.h"

struct ContactData;
class RigidBody {
public:

	RigidBody(const float mass, const glm::mat3 tensorBody, const glm::vec3& position = glm::vec3(), const glm::fquat& orientation = glm::angleAxis(glm::radians(0.f), glm::vec3(0, 0, 1)), const glm::vec3& velocity = glm::vec3(), const glm::vec3& omega = glm::vec3());

	void integrate(float duration);

	void syncColliders();

	//point is given in local space
	virtual void applyForce(const glm::vec3& point, const glm::vec3& force);

	void applyForce(const glm::vec3& force);

	point transform(const point& pt) const;

	point transformInverse(const point& pt) const;

	inline const glm::vec3& getPosition() const { return position; }

	inline const glm::mat3& getRotation() const { return rotation; }

	inline const glm::mat3& getRotationT() const { return rotationT; }

	inline const glm::vec3& getAngularVelocity() const { return omega; }

	inline const glm::vec3& getVelocity() const { return velocity; }

	//inline const glm::vec3& getAngularMomentum() const { return angularMomentum; }

	//inline const glm::vec3& getLinearMomentum() const { return linearMomentum; }

	inline const glm::fquat& getOrientation() const { return orientation; }

	inline const std::unique_ptr<BoundingSphere>& const getCollider1() { return collider1; }

	inline const std::unique_ptr<OBB>& const getCollider2() { return collider2; }

	//point in world space
	const glm::vec3 getParticleVelocity(const glm::vec3& point) const {
		return velocity + glm::cross(omega, transform(point));
	}
	
	inline const float getInverseMass() const{ return invMass; }

	inline const float getMass() const { return 1 / invMass; }

	inline const glm::mat3& getInverseTensor() const { return invTensor; }

	inline virtual const glm::mat4 getModel(glm::vec3 scaler) const { return glm::identity<glm::mat4>(); }
	
	inline void kill() { alive = 0; }

	inline bool isAwake() { return awake; }

	inline bool isDead() { return !alive; }

	virtual void render() const {};
	
	friend void applyImpulse(ContactData* contact, float epsilon);

	friend void resolveInterpentration(ContactData* contact);
	
protected:
	glm::vec3 position;
	glm::fquat orientation;
	glm::vec3 velocity;
	glm::vec3 omega;
	//glm::vec3 linearMomentum;
	//glm::vec3 angularMomentum;

	glm::mat3 invTensor;
	glm::mat3 rotation;
	glm::mat3 rotationT;


	//constant variable
	glm::mat3 tensorBody;
	glm::mat3 invTensorBody;
	float invMass;

	glm::vec3 forceAccum;
	glm::vec3 torqueAccum;

	//int colliderType;
	std::unique_ptr<BoundingSphere> collider1;
	std::unique_ptr<OBB> collider2;
	
	bool awake;
	bool alive;

	void calcDerivedQuantities();

	inline void clearAccum() { forceAccum[0] = forceAccum[1] = forceAccum[2] = torqueAccum[0] = torqueAccum[1] = torqueAccum[2] = 0; }
};

class SolidSphere : public RigidBody{
public:
	SolidSphere(const float mass, const float radius, const glm::vec3 position = glm::vec3(), const glm::fquat orientation = glm::angleAxis(glm::radians(0.f), glm::vec3(0, 0, 1)), const glm::vec3 velocity = glm::vec3(), const glm::vec3 omega = glm::vec3());

	virtual void applyForce(const glm::vec3& point, const glm::vec3& force) override;
	
	inline float getRadius() { return radius; }
	
	inline virtual const glm::mat4 getModel(glm::vec3 scaler) const override {
		glm::mat4 model = glm::scale(glm::identity<glm::mat4>(), scaler);
		model = glm::translate(model, position) * glm::mat4(rotation);
		model = glm::scale(model, glm::vec3(radius));
		return model;
	};

	virtual void render() const override;
	static void generateVertices();

private:
	const float radius;

	inline static glm::mat3 generateTensor(float mass, float radius);

	static std::vector<float> vertices;
	static std::vector<unsigned int> indices;
	std::unique_ptr<renderer::vertexbuffer> VBO;
	std::unique_ptr<renderer::vertexarray> VAO;
	std::unique_ptr<renderer::indexbuffer> IBO;
};

class SolidCuboid : public RigidBody{
public:
	SolidCuboid(const float mass, const glm::vec3& extens, const glm::vec3 position = glm::vec3(), const glm::fquat orientation = glm::angleAxis(glm::radians(0.f), glm::vec3(0, 0, 1)), const glm::vec3 velocity = glm::vec3(), const glm::vec3 omega = glm::vec3());

	virtual void applyForce(const glm::vec3& point, const glm::vec3& force) override;

	inline const glm::vec3& getExtents() { return extents; }
	
	inline virtual const glm::mat4 getModel(glm::vec3 scaler) const override {
		glm::mat4 model = glm::scale(glm::identity<glm::mat4>(), scaler);
		model = glm::translate(model, position) * glm::mat4(rotation);
		model = glm::scale(model, extents);
		return model;
	};

	virtual void render() const override;

private:
	glm::vec3 extents;
	inline static glm::mat3 generateTensor(float mass, const glm::vec3& extents);

	static std::vector<float> vertices;
	static std::vector<unsigned int> indices;
	std::unique_ptr<renderer::vertexbuffer> VBO;
	std::unique_ptr<renderer::vertexarray> VAO;
	std::unique_ptr<renderer::indexbuffer> IBO;
};

struct ContactData {
	RigidBody* A;
	RigidBody* B;
	const CollisionManifold* M;

	ContactData(RigidBody* A, RigidBody* B, const CollisionManifold* M) : A(A), B(B), M(M) {}
	~ContactData() { delete M; }
};

void applyImpulse(ContactData* contact, float epsilon);

void resolveInterpentration(ContactData* contact);
#endif // !RIGID_BODY_H
