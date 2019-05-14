#ifndef TESTCLEARCOLOR_H
#define TESTCLEARCOLOR_H
#include "Test.h"
#include "basic.h"
#include "vendor\imgui\imgui.h"
namespace test {
	class TestClearColor : public Test {
		TestClearColor();
		virtual void OnUpdate(float deltaTime) override;
		virtual void OnRender() override;
		virtual void OnImGuiRender() override;
	private:
		float m_ClearColor[4];
	};
}
#endif // !TESTCLEARCOLOR_H
