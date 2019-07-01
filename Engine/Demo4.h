#ifndef DEMO4_H
#define DEMO4_H
#include "Test.h"
#include "RigidBody.h"
#include "forceGen.h"
#include "shader.h"
#include "vertexarray.h"
#include "vertexbuffer.h"
#include "indexbuffer.h"
#include "camera.h"
#include "skybox.h"
#include "texture.h"
#include "OcTree.h"
#include "MousePicker.h"
#include "Joint.h"
extern renderer::camera camera;
extern const unsigned int SCR_WIDTH;
extern const unsigned int SCR_HEIGHT;
extern int Ndir;
namespace test {
	class Demo4 :public Test {
	public:
		Demo4();
		void init();
		void reset();
		~Demo4();
		void OnRender() override;
		void OnImGuiRender() override;
	private:
		bool update = 1;
		bool debugRender = 0;
		//initial data
		float mass = 1;
		float rad = 1;
		Material material;
		glm::vec3 pos = glm::vec3(0, 2, 0);
		glm::vec3 extents = glm::vec3(1.f);
		glm::vec3 force = glm::vec3(0.f);
		glm::vec3 pt = glm::vec3(0.f);
		glm::vec3 axis = glm::vec3(0, 0, 1);
		glm::vec3 Jpt = glm::vec3(0.f);
		float theta = 0;
		//modify data
		glm::vec3 bodyPos;
		float bodyMass;
		glm::vec3 bodyExtents;
		float bodyRad;
		Material bodyMaterial;
		float t = 0.5f;
		//World data
		GravityForce* Earth = GravityForce::EarthGravity();
		GravityForce* Moon = GravityForce::moonGravity();
		GravityForce* Saturn = GravityForce::saturnGravity();
		GravityForce* Jupiter = GravityForce::jupiterGravity();
		static const char* gravity[4];
		static const char* currentgravity;
		bool read = 1;
		MousePicker picker;
		RigidBody* modifyBody;
		SolidCuboid* cuboid;
		SolidSphere* sphere;
		std::list<RigidBody*> bodies;
		OcTree* tree;
		ForceRegistry registry;
		JointRegistry Jregistry;
		renderer::shader* mainShader;
		renderer::skybox* cubeMap;
		DirLight light;
	};
}
#endif // !DEMO4_H