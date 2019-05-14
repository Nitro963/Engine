#ifndef RIGID_BODY_H
#define RIGID_BODY_H
#include "glm\gtc\matrix_transform.hpp"
#include "glm\gtx\quaternion.hpp"
#include "glm\glm.hpp"
#include "glm\gtc\matrix_inverse.hpp"

#include <vector>

#include "vertexbuffer.h"
#include "vertexarray.h"
#include "indexbuffer.h"
#include "Geometry.h"
#include "OBB.h"
#include "BoundingSphere.h"
#include "Plane.h"
#include "texture.h"

inline glm::mat3 star(const glm::vec3 ve) {
	glm::mat3 m(0.f);
	m[0][1] = -ve.z;
	m[0][2] = ve.y;
	m[1][0] = ve.z;
	m[1][2] = -ve.x;
	m[2][0] = -ve.y;
	m[2][1] = ve.x;
	return m;
}

class RigidBody {
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

	//constant variable
	const glm::mat3 tensorBody;
	const glm::mat3 invTensorBody;
	const float invMass;

	glm::vec3 forceAccum;
	glm::vec3 torqueAccum;
public:

	RigidBody(const float mass, const glm::mat3 tensorBody, const glm::vec3& position = glm::vec3(), const glm::fquat& orientation = glm::angleAxis(glm::radians(0.f), glm::vec3(0, 0, 1)), const glm::vec3& velocity = glm::vec3(), const glm::vec3& omega = glm::vec3());

	void integrate(float duration);

	void calcDerivedQuantities();

	//point is given in local space
	virtual void applyForce(const glm::vec3& point, const glm::vec3& force);

	void applyForce(const glm::vec3& force);

	point transformWorld(const point& pt) const;

	point transformLocal(const point& pt) const;

	inline void clearAccum() { forceAccum[0] = forceAccum[1] = forceAccum[2] = torqueAccum[0] = torqueAccum[1] = torqueAccum[2] = 0; }

	inline const glm::vec3& getPosition() const { return position; }

	inline const glm::mat3& getRotation() const { return rotation; }

	inline const glm::mat3& getRotationT() const { return rotationT; }

	inline const glm::vec3& getAngularVelocity() const { return omega; }

	inline const glm::vec3& getVelocity() const { return velocity; }

	inline const glm::vec3& getAngularMomentum() const { return angularMomentum; }

	inline const glm::vec3& getLinearMomentum() const { return linearMomentum; }

	inline const glm::fquat& getOrientation() const { return orientation; }
	//point in world space
	const glm::vec3& getParticleVelocity(const glm::vec3& point) const {
		return velocity + glm::cross(omega, transformWorld(point));
	}
	
	inline const float getInverseMass() const{ return invMass; }

	inline const float getMass() const { return 1 / invMass; }

	inline const glm::mat3& getInverseTensor() const { return invTensor; }

	inline void setAngularMomentum(const glm::vec3& L) { angularMomentum = L; }

	inline void setLinearMomentum(const glm::vec3& P) { linearMomentum = P; }

	inline void setVelocity(const glm::vec3& V) { velocity = V; }

	inline void setAngularVelocity(const glm::vec3& O) { omega = O; }
};

struct Contact {
	RigidBody *a  // body containing vertex 
		, *b; // body containing face
	glm::vec3 point, // world-space vertex position
		normal, // outwards point face normal
		edgeA, //edge direction of A
		edgeB; //edge dirction of B
	bool vertexFace; //true if vertex/face contact
};

class SolidCuboid;
class SolidSphere : public RigidBody{
private:
	const float radius;
	
	inline static glm::mat3 generateTensor(float mass, float radius);
	static void generateVertices();

	static std::vector<float> vertices;
	static std::vector<uint32_t> indices;

	static renderer::vertexbuffer* VBO;
	static renderer::vertexarray* VAO;
	static renderer::indexbuffer* IBO;
public:
	SolidSphere(const float mass, const float radius, const glm::vec3& position = glm::vec3(), const glm::fquat& orientation = glm::angleAxis(glm::radians(0.f) ,glm::vec3(0 ,0 ,1)), const glm::vec3& velocity = glm::vec3(), const glm::vec3& omega = glm::vec3());
	
	virtual void applyForce(const glm::vec3& point, const glm::vec3& force) override;
	
	inline float getRadius() { return radius; }

	bool isColliding(SolidSphere* other ,Contact* contactData);

	bool isColliding(SolidCuboid* other, Contact* contactData);

	void draw();
};

class SolidCuboid : public RigidBody{
private:
	glm::vec3 extents;
	inline static glm::mat3 generateTensor(float mass, const glm::vec3& extents);

	static std::vector<float> vertices;
	static std::vector<uint32_t> indices;

	static renderer::vertexbuffer* VBO;
	static renderer::vertexarray* VAO;
	static renderer::indexbuffer* IBO;
public:
	SolidCuboid(const float mass, const glm::vec3& extens, const glm::vec3& position = glm::vec3(), const glm::fquat& orientation = glm::angleAxis(glm::radians(0.f), glm::vec3(0, 0, 1)), const glm::vec3& velocity = glm::vec3(), const glm::vec3& omega = glm::vec3());

	virtual void applyForce(const glm::vec3& point, const glm::vec3& force) override;

	inline const glm::vec3& getExtents() { return extents; }

	//bool isColliding(const SolidCuboid& other);

	void draw();
};

class Floor : public RigidBody{
private:
	std::vector<float> vertices;
	std::vector<uint32_t> indices;
	renderer::vertexbuffer* VBO;
	renderer::vertexarray* VAO;
	renderer::indexbuffer* IBO;
	
	plane collider;

	static Floor* object;
	
	inline static glm::mat4 generateTensor() { glm::mat4 ret(0.f); ret[0][0] = ret[1][1] = ret[2][2] = 1e9 + 9; return ret; }
	Floor(const glm::vec3& position = glm::vec3(0 ,-5 ,0)) :RigidBody(1e9 + 9, generateTensor(), position) ,collider(point(200 ,-10 ,200) ,point(-200 ,-10 ,-200) ,point(-200 ,-10 ,200)){
		
		vertices = {
			//positions                 texcoord
			-200.f, -10.f, -200.f,			0 ,0,
			200.f, -10.f, -200.f,			1 ,0,
			200.f, -10.f, 200.f,			1 ,1,
			-200.f, -10.f, 200.f,			0 ,1
		};
		indices = {
			0 ,1 ,2,
			2 ,3 ,0
		};
		VBO = new renderer::vertexbuffer(&vertices[0], sizeof(float) * vertices.size());
		IBO = new renderer::indexbuffer(&indices[0], indices.size());
		renderer::vertexbufferlayout layout;
		layout.push<float>(3);
		layout.push<float>(2);
		VAO = new renderer::vertexarray();
		VAO->addbuffer(*VBO, layout);
		
	}
public:
	static Floor* getInstance();

	inline void draw() {
		VAO->bind();
		IBO->bind();
		VAO->bind();
		IBO->bind();
		GLCall(glDrawElements(GL_TRIANGLES, IBO->getcount(), GL_UNSIGNED_INT, NULL));
		VAO->unbind();
		IBO->unbind();
	}
	
	bool isColliding(SolidSphere* other, Contact* contactData);

	bool isColliding(SolidCuboid* other, Contact* contactData);
};

bool isColliding(Contact* c);

void handelCollision(Contact *c, float epsilon);
#endif // !RIGID_BODY_H
