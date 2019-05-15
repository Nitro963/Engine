#include "TestClearColor.h"

namespace test {
	TestClearColor::TestClearColor() : m_ClearColor{ 0.4f ,0.2f ,0.8f } {}

	void TestClearColor::OnUpdate(float deltaTime){}

	void TestClearColor::OnRender(){
		GLCall(glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT));
		GLCall(glClearColor(m_ClearColor[0], m_ClearColor[1], m_ClearColor[2], m_ClearColor[3]));
	}

	void TestClearColor::OnImGuiRender(){
		ImGui::ColorEdit4("clear color", m_ClearColor);
	}
	
}