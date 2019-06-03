#ifndef TESTFLOOR_H
#define TESTFLOOR_H
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
	class TestFloor :public Test{
	public:
		TestFloor();
		~TestFloor();
		void OnRender() override;
		void OnImGuiRender() override;
	private:
		SolidCuboid* cube;
		SolidSphere*  sphere;
		SolidSphere* sphere1;
		renderer::skybox* cubeMap;
		renderer::shader* mainShader;
		glm::vec3 force;
		glm::vec3 point;
	};
}
#endif // !TESTFLOOR_H
