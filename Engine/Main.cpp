#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include "glm\glm.hpp"
#include "vendor\imgui\imgui.h"
#include "vendor\imgui\imgui_impl_glfw.h"
#include "vendor\imgui\imgui_impl_opengl3.h"

#include <iostream>
#include <vector>

#include "RigidBody.h"
#include "forceGen.h"
#include "shader.h"
#include "vertexarray.h"
#include "vertexbuffer.h"
#include "indexbuffer.h"
#include "camera.h"
#include "skybox.h"
#include "texture.h"
#include "Test.h"
#include "TestClearColor.h"
#include "TestFloor.h"
#include "TestQubes.h"

#pragma region auxiliary
renderer::camera camera(glm::vec3(0, 1, 3));

const unsigned int SCR_WIDTH = 1200;
const unsigned int SCR_HEIGHT = 720;
int slot0 = 0;
int slot1 = 1;
float lastX = SCR_WIDTH / 2.0f;
float lastY = SCR_HEIGHT / 2.0f;
bool firstMouse = true;
float deltaTime = 0.0f;
float lastFrame = 0.0f;
bool pause = false;
bool ctrl = false;

void framebufferSizeCallback(GLFWwindow* window, int width, int height) {
	//glViewport((width - 720) / 2, (height - 720) / 2, width, 720);
	glViewport(0, 0, width, height);
}

void mouseCallback(GLFWwindow* window, double xpos, double ypos) {
	if (!ctrl)
		return;
	if (firstMouse) {
		lastX = xpos;
		lastY = ypos;
		firstMouse = false;
	}

	float xoffset = xpos - lastX;
	float yoffset = lastY - ypos;

	lastX = xpos;
	lastY = ypos;

	camera.process_mouse_movement(xoffset, yoffset);
}

void scrollCallback(GLFWwindow* window, double xoffset, double yoffset) {
	camera.process_mouse_scroll(yoffset);
}

void processInput(GLFWwindow *window) {
	if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
		glfwSetWindowShouldClose(window, true);

	if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
		camera.process_keyboard(renderer::FORWARD, deltaTime);
	if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
		camera.process_keyboard(renderer::BACKWARD, deltaTime);
	if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
		camera.process_keyboard(renderer::LEFT, deltaTime);
	if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
		camera.process_keyboard(renderer::RIGHT, deltaTime);

	if (glfwGetKey(window, GLFW_KEY_P) == GLFW_PRESS)
		pause = true;

	if (glfwGetKey(window, GLFW_KEY_C) == GLFW_PRESS)
		pause = false;

	if (glfwGetKey(window, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS)
		ctrl = true;
	else
		ctrl = false, firstMouse = true;
}

void glfwErrorCallback(int error, const char* description) {
	fprintf(stderr, "Glfw Error %d: %s\n", error, description);
}

#pragma endregion auxiliary

int main() {

#pragma region Init
	GLFWwindow* window;
	if (!glfwInit())
		return -1;

	const char* glslVersion = "#version 330 core";
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);

	window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "Physics Engine", NULL, NULL);
	if (!window) {
		glfwTerminate();
		return -1;
	}

	glfwMakeContextCurrent(window);
	glfwSetFramebufferSizeCallback(window, framebufferSizeCallback);
	glfwSetCursorPosCallback(window, mouseCallback);
	glfwSetScrollCallback(window, scrollCallback);
	glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
	glfwSetErrorCallback(glfwErrorCallback);

	if (glewInit())
		throw std::exception("Failed to initialize GLEW\n");


	GLCall(glEnable(GL_DEPTH_TEST));
	GLCall(glEnable(GL_BLEND));
	GLCall(glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA));


	IMGUI_CHECKVERSION();

	ImGui::CreateContext();

	ImGui_ImplGlfw_InitForOpenGL(window, true);
	ImGui_ImplOpenGL3_Init(glslVersion);

	ImGui::StyleColorsDark();
#pragma endregion Init

	test::Test* currentTest = nullptr;
	test::TestMenu* testMenu = new test::TestMenu(currentTest);
	currentTest = testMenu;

	test::TestClearColor test;
	testMenu->registerTest<test::TestClearColor>("clear color");
	testMenu->registerTest<test::TestFloor>("floor");
	testMenu->registerTest<test::Demo>("Frashan");

	while (!glfwWindowShouldClose(window)) {
		glfwSwapInterval(1);
		float currentFrame = glfwGetTime();
		deltaTime = currentFrame - lastFrame;
		lastFrame = currentFrame;

		ImGui_ImplOpenGL3_NewFrame();
		ImGui_ImplGlfw_NewFrame();
		ImGui::NewFrame();

#pragma region Imgui

		ImGui::Begin("Debug");

		if (currentTest) {
			if (currentTest != testMenu && ImGui::Button("back")) {
				delete currentTest;
				currentTest = testMenu;
			}
			currentTest->OnRender();
			currentTest->OnImGuiRender();
		}

		ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);

		ImGui::End();

#pragma endregion Imgui

		ImGui::Render();
		ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

		glfwSwapBuffers(window);

		glfwPollEvents();

		processInput(window);
	}
	
	ImGui::DestroyContext();
	glfwTerminate();
	_exit(0);
}