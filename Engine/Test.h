#ifndef TEST_H
#define TEST_H
#include <vector>
#include <string>
#include <functional>
#include <iostream>
#include "vendor\imgui\imgui.h"
#include "basic.h"
#include "skybox.h"
namespace test {
	class Test {
	public:
		Test() {}
		virtual ~Test() {}
		virtual void OnUpdate(float deltaTime) {}
		virtual void OnRender() {}
		virtual void OnImGuiRender() {}
	};
	class TestMenu : public Test {
	public:
		TestMenu(Test*& currentTestPointer);
		void OnRender() override;
		void OnImGuiRender() override;
		template<typename T>
		void registerTest(const std::string& name) {
			std::cout << "Registering test " << name << '\n';
			m_tests.push_back(std::make_pair(name, []() {return new T(); }));
		}
	private:
		std::vector<std::pair<std::string, std::function<Test*()>>> m_tests;
		Test*& m_CurrentTest;
		renderer::skybox* cubeMap;
	};
}
#endif // !TEST_H
