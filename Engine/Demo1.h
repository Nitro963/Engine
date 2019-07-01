#ifndef SPHEREDEMO_H
#define SPHEREDEMO_H
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
	class Demo1 :public Test {
	public:
		Demo1();
		void init();
		void reset();
		~Demo1();
		void OnRender() override;
		void OnImGuiRender() override;
	private:
		bool update = 0;
		bool debugRender = 0;
		float mass = 1;
		float rad = 1;
		glm::vec3 pos = glm::vec3(0, 3, 0);
		glm::vec3 extents = glm::vec3(1.f);
		std::list<RigidBody*> bodies;
		OcTree* tree;
		ForceRegistry registry;
		renderer::shader* mainShader;
	};
}
#endif // !SPHEREDEMO_H
