#ifndef TESTQUBE_H
#define TESTQUBE_H
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

extern renderer::camera camera;

namespace test {
	class Demo :public Test {
	public:
		Demo();
		~Demo();
		void OnRender() override;
		void OnImGuiRender() override;
	private:
		std::vector<RigidBody*> bodies;
		renderer::shader* mainShader;
		glm::vec3 force;
		glm::vec3 point;
	};
}
#endif // !TESTQUBE_H
