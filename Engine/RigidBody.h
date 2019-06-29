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

#define VELOCITYLIMIT 1e-4

struct Material {
	float shininess;
	float epsilon;
	float mu;
	float muDynamic;
	Material() : shininess(32), epsilon(0.7), mu(0.5) ,muDynamic(0.3) {}
	Material(const float& shininess, const float& epsilon, const float& mu) : shininess(shininess), epsilon(epsilon), mu(mu){}
};

class RigidBody {
public:

	static float damping;

	RigidBody(const float mass, const glm::mat3 tensorBody, const Material& bodyMaterial = Material(), const glm::vec3& position = glm::vec3(), const glm::fquat& orientation = glm::angleAxis(glm::radians(0.f), glm::vec3(0, 0, 1)), const glm::vec3& velocity = glm::vec3(), const glm::vec3& omega = glm::vec3());

	void integrate(float duration);

	void syncColliders();

	//point is given in local space
	virtual void applyImpulse(const glm::vec3& point, const glm::vec3& impulse);

	void applyImpulse(const glm::vec3& impulse);

	point transform(const point& pt) const;

	point transformInverse(const point& pt) const;

	inline const glm::vec3& getPosition() const { return position; }

	inline const glm::mat3& getRotation() const { return rotation; }

	inline const glm::vec3& getAngularVelocity() const { return omega; }

	inline const glm::vec3& getVelocity() const { return velocity; }

	inline const Material& getMaterial() const { return bodyMaterial; }

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

	inline void translate(const glm::vec3& translation) { position += translation; }

	inline void rotate(const glm::fquat& rotation) { orientation = rotation * orientation; }

	inline void setPosition(const glm::vec3& position) {
		this->position = position;
		syncColliders();
	}

	inline void setVelocity(const glm::vec3& velocity) {
		this->velocity = velocity;
	}

	inline void setAngularVelocity(const glm::vec3& omega) {
		this->omega = omega;
	}

	inline void setOrientation(const glm::fquat& orientation) {
		this->orientation = orientation;
		calcDerivedQuantities();
	}

	inline void setMaterial(const Material& bodyMaterial) {
		this->bodyMaterial = bodyMaterial;
	}

	inline virtual const glm::mat4 getModel(const glm::vec3& scaler) const { return glm::identity<glm::mat4>(); }

	inline void kill() { alive = 0; }

	inline bool isAwake() const { return awake; }

	inline bool isDead() const { return !alive; }

	virtual void render() const {};
	
	friend void resolveContact(struct ContactData* contact);

	friend void resolveInterpentration(struct ContactData* contact);
protected:
	glm::vec3 position;
	glm::fquat orientation;
	glm::vec3 velocity;
	glm::vec3 omega;
	//glm::vec3 linearMomentum;
	//glm::vec3 angularMomentum;

	glm::mat3 invTensor;
	glm::mat3 rotation;

	//constant variable
	glm::mat3 tensorBody;
	glm::mat3 invTensorBody;
	float invMass;
	Material bodyMaterial;

	//int colliderType;
	std::unique_ptr<BoundingSphere> collider1;
	std::unique_ptr<OBB> collider2;
	
	bool awake;
	bool alive;

	void calcDerivedQuantities();
};

inline bool isDead(RigidBody*& body) {
	return body->isDead();
}

class SolidSphere : public RigidBody{
public:
	SolidSphere(const float mass, const float radius, const Material& bodyMaterial = Material(), const glm::vec3 position = glm::vec3(), const glm::fquat orientation = glm::angleAxis(glm::radians(0.f), glm::vec3(0, 0, 1)), const glm::vec3 velocity = glm::vec3(), const glm::vec3 omega = glm::vec3());

	virtual void applyImpulse(const glm::vec3& point, const glm::vec3& Impulse) override;
	
	inline float getRadius() { return radius; }
	
	inline void update(float mass, float radius) {
		invMass = 1 / mass;
		tensorBody = generateTensor(mass, radius);
		invTensorBody = glm::inverse(tensorBody);
		this->radius = radius;
		collider1->update(position, radius);
	}

	inline virtual const glm::mat4 getModel(const glm::vec3& scaler) const override {
		glm::mat4 model = glm::scale(glm::identity<glm::mat4>(), scaler);
		model = glm::translate(model, position) * glm::mat4(rotation);
		model = glm::scale(model, glm::vec3(radius));
		return model;
	};

	virtual void render() const override;
	static void generateVertices();

private:
	float radius;

	inline static glm::mat3 generateTensor(float mass, float radius);

	static std::vector<float> vertices;
	static std::vector<unsigned int> indices;
	std::unique_ptr<renderer::vertexbuffer> VBO;
	std::unique_ptr<renderer::vertexarray> VAO;
	std::unique_ptr<renderer::indexbuffer> IBO;
};

class SolidCuboid : public RigidBody{
public:
	SolidCuboid(const float mass, const glm::vec3& extens, const Material& bodyMaterial = Material(), const glm::vec3 position = glm::vec3(), const glm::fquat orientation = glm::angleAxis(glm::radians(0.f), glm::vec3(0, 0, 1)), const glm::vec3 velocity = glm::vec3(), const glm::vec3 omega = glm::vec3());

	virtual void applyImpulse(const glm::vec3& point, const glm::vec3& impulse) override;

	inline const glm::vec3& getExtents() { return extents; }
	
	inline void update(float mass, const glm::vec3 extents) {
		invMass = 1 / mass;
		tensorBody = generateTensor(mass, extents);
		invTensorBody = glm::inverse(tensorBody);
		this->extents = extents;
		collider2->update(position, rotation, extents * 0.5f);
	}

	inline virtual const glm::mat4 getModel(const glm::vec3& scaler) const override {
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

void resolveContact(ContactData* contact);

void resolveInterpentration(ContactData* contact);
#endif // !RIGID_BODY_H
