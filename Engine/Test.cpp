#include "Test.h"
namespace test {
	TestMenu::TestMenu(Test *& currentTestPointer):m_CurrentTest(currentTestPointer){
		std::vector<std::string> faces{
			"res/Textures/SkyBoxs/mp_sky/right.jpg",
			"res/textures/skyboxs/mp_sky/left.jpg",
			"res/textures/skyboxs/mp_sky/top.jpg",
			"res/textures/skyboxs/mp_sky/bottom.jpg",
			"res/textures/skyboxs/mp_sky/front.jpg",
			"res/textures/skyboxs/mp_sky/back.jpg"
		};
		cubeMap = new renderer::skybox(faces);
	}
	void TestMenu::OnRender(){
		GLCall(glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT));
		GLCall(glClearColor(0.f, 0.f, 0.f, 0.f));
		cubeMap->render();
	}
	void TestMenu::OnImGuiRender(){
		for (auto& test : m_tests)
			if (ImGui::Button(test.first.c_str()))
				m_CurrentTest = test.second();
	}
}