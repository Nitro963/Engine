#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include "glm\glm.hpp"
#include "vendor\imgui\imgui.h"
#include "vendor\imgui\imgui_impl_glfw.h"
#include "vendor\imgui\imgui_impl_opengl3.h"

#include <iostream>

#include "PhysicsEngine.h";

void framebuffer_size_callback(GLFWwindow* window, int width, int height) {
	glViewport((width - 720) / 2, (height - 720) / 2, 720, 720);
}

int main(void){
	GLFWwindow* window;

	if (!glfwInit())
		return -1;
	const char* glslVersion = "#version 450";
	//glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	//glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);

	window = glfwCreateWindow(720, 720, "Hello World", NULL, NULL);
	if (!window){
		glfwTerminate();
		return -1;
	}
	glfwMakeContextCurrent(window);
	glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);

	if (glewInit())
		throw std::exception("Failed to initialize GLEW\n");

	IMGUI_CHECKVERSION();

	ImGui::CreateContext();

	ImGui_ImplGlfw_InitForOpenGL(window, true);
	ImGui_ImplOpenGL3_Init(glslVersion);

	ImGui::StyleColorsDark();

	physicsEngine pEngine(0.1);
	pEngine.addObject(physicalObject(glm::vec3(0, 0, 0), 1));
	pEngine.addObject(physicalObject(glm::vec3(0, 1e6 * 6.371 * -1, 0), 1e24 * 5.972));
	glm::vec4 clearColor(0 ,0 ,0 ,1.f);
	while (!glfwWindowShouldClose(window)){
		glClear(GL_COLOR_BUFFER_BIT);
		glfwSwapInterval(1);

		ImGui_ImplOpenGL3_NewFrame();
		ImGui_ImplGlfw_NewFrame();
		ImGui::NewFrame();

#pragma region Imgui

		ImGui::Begin("Debug");

		ImGui::ColorEdit3("clear color", (float*)&clearColor);

		ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);

		ImGui::End();

#pragma endregion Imgui

		glLoadIdentity();
		glScaled(0.1, 0.1, 0);
		pEngine.simulate(60);
		glBegin(GL_POLYGON);
		for (double i = 0; i < 2 * glm::acos(-1); i+= 0.05)
			glVertex2d(cos(i) + pEngine.getObjects().front().getPosition().x, sin(i) + pEngine.getObjects().front().getPosition().y);
		glEnd();


		ImGui::Render();
		ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
		glClearColor(clearColor.x, clearColor.y, clearColor.z, clearColor.w);

		glfwSwapBuffers(window);

		glfwPollEvents();
	}

	glfwTerminate();
	return 0;
}