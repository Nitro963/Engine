#ifndef DEMO2_H
#define DEMO2_H
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
extern renderer::camera camera;
extern const unsigned int SCR_WIDTH;
extern const unsigned int SCR_HEIGHT;

namespace test {
	class Demo2 :public Test {
	public:
		Demo2();
		void init();
		void reset();
		~Demo2();
		void OnRender() override;
		void OnImGuiRender() override;
	private:
		bool update;
		bool debugRender = 0;
		float mass = 1;
		float rad = 1;
		glm::vec3 pos = glm::vec3(0, 3, 0);
		glm::vec3 extents = glm::vec3(1.f);
		glm::vec3 force = glm::vec3(0.f);
		glm::vec3 pt = glm::vec3(0.f);
		glm::vec3 axis = glm::vec3(0, 0, 1);
		float theta = 0;
		bool read = 1;
		float t = 0.5f;
		MousePicker picker;
		RigidBody* modifyBody;
		SolidCuboid* cuboid;
		SolidSphere* sphere;
		std::list<RigidBody*> bodies;
		OcTree* tree;
		ForceRegistry registry;
		renderer::shader* mainShader;
	};
}
#endif // !DEMO2_H
