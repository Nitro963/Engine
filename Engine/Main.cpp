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

#pragma region auxiliary
renderer::camera camera(glm::vec3(0, 0, 3));

const unsigned int SCR_WIDTH = 1200;
const unsigned int SCR_HEIGHT = 720;
int slot0 = 0;
int slot1 = 1;
float lastX = SCR_WIDTH / 2.0f;
float lastY = SCR_HEIGHT / 2.0f;
bool first_mouse = true;
float delta_time = 0.0f;
float last_frame = 0.0f;
bool pause = false;
bool ctrl = false;

void framebufferSizeCallback(GLFWwindow* window, int width, int height) {
	//glViewport((width - 720) / 2, (height - 720) / 2, width, 720);
	glViewport(0, 0, width, height);
}

void mouseCallback(GLFWwindow* window, double xpos, double ypos) {
	if (!ctrl)
		return;
	if (first_mouse) {
		lastX = xpos;
		lastY = ypos;
		first_mouse = false;
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
		camera.process_keyboard(renderer::FORWARD, delta_time);
	if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
		camera.process_keyboard(renderer::BACKWARD, delta_time);
	if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
		camera.process_keyboard(renderer::LEFT, delta_time);
	if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
		camera.process_keyboard(renderer::RIGHT, delta_time);

	if (glfwGetKey(window, GLFW_KEY_P) == GLFW_PRESS)
		pause = true;

	if (glfwGetKey(window, GLFW_KEY_C) == GLFW_PRESS)
		pause = false;

	if (glfwGetKey(window, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS)
		ctrl = true;
	else
		ctrl = false, first_mouse = true;
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

	glm::vec4 clearColor(0, 0, 0, 1.f);


	{
		renderer::shader shader("res/shaders/basic.shader");
		shader.use();
		SolidSphere* sphere = new SolidSphere(1, 0.2, glm::vec3(0, 0.5 + 0.3, 0));
		SolidSphere* sphere1 = new SolidSphere(1, 0.5 ,glm::vec3(1 ,0 ,0));
		glm::vec3 pt(0.5, 0, 0);
		glm::vec3 pt1(0, 0.5, 0);
		glm::vec3 f(0, 9.8, 0);
		glm::vec3 f1(0, 0, 9.8);

		SolidCuboid* cube = new SolidCuboid(1, glm::vec3(1, 1, 1));
		ForceRegistry registry;
		glm::vec3 anchor(0.f ,0.5 + 0.3 ,0);
		glm::vec3 connectionPoint(0, 0.5, 0);
		glm::vec3 initPos = cube->transformLocal(connectionPoint);
		//registry.add(cube, new FakeSpring(&anchor ,connectionPoint ,&initPos ,20 ,0.1));
		//registry.add(cube, new GravityForce(glm::vec3(0, -9.8, 0)));

		renderer::shader cubeMapShader("res/shaders/cubeMap.shader");
		
		//std::vector<std::string> faces{
		//	"res/Textures/SkyBoxs/mp_sky/right.jpg",
		//	"res/textures/skyboxs/mp_sky/left.jpg",
		//	"res/textures/skyboxs/mp_sky/top.jpg",
		//	"res/textures/skyboxs/mp_sky/bottom.jpg",
		//	"res/textures/skyboxs/mp_sky/front.jpg",
		//	"res/textures/skyboxs/mp_sky/back.jpg"
		//};
		std::vector<std::string> faces{
			"res/Textures/SkyBoxs/New folder/right.jpg",
			"res/textures/skyboxs/New folder/left.jpg",
			"res/textures/skyboxs/New folder/top.jpg",
			"res/textures/skyboxs/New folder/bottom.jpg",
			"res/textures/skyboxs/New folder/front.jpg",
			"res/textures/skyboxs/New folder/back.jpg"
		};

		renderer::skybox cubeMap(faces);
		
		renderer::shader floorShader("res/shaders/floor.shader");
		renderer::texture tex("res/textures/floor/bricks.jpg");

		float time = 0.f;
		while (!glfwWindowShouldClose(window)) {
			glfwSwapInterval(1);
			float current_frame = glfwGetTime();
			delta_time = current_frame - last_frame;
			last_frame = current_frame;

			GLCall(glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT));
			ImGui_ImplOpenGL3_NewFrame();
			ImGui_ImplGlfw_NewFrame();
			ImGui::NewFrame();

			glm::mat4 view = camera.get_view_matrix();
			glm::mat4 projection = glm::perspective(glm::radians(camera.get_zoom()), (float)SCR_WIDTH / SCR_HEIGHT, 0.1f, 100.f);

#pragma region Imgui

			ImGui::Begin("Debug");

			ImGui::InputFloat3("Anchor", &anchor[0]);

			ImGui::InputFloat3("offset", &connectionPoint[0]);

			ImGui::InputFloat3("point", &pt1[0]);

			ImGui::InputFloat3("force", &f1[0]);

			//if (ImGui::Button("apply force"))
			//	sphere->applyForce(pt, f);
			//
			//if (ImGui::Button("negate force")) 
			//	sphere->applyForce(pt, -f);
			
			if (ImGui::Button("apply force Cube"))
				cube->applyForce(pt1, f1);

			if (ImGui::Button("negate force Cube"))
				cube->applyForce(pt1, -f1);

			ImGui::ColorEdit3("clear color", (float*)&clearColor);

			ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);

			ImGui::Text("Application time %.3f", time);

			ImGui::End();

#pragma endregion Imgui

			glm::mat4 model = glm::scale(glm::mat4(1.f), glm::vec3(0.5, 0.5, 0.5));
			model = glm::translate(model, sphere->getPosition()) * glm::mat4(sphere->getRotation());
			model = glm::scale(model, glm::vec3(sphere->getRadius()));

			shader.use();
			shader.set_uniform<glm::mat4>("view", view);
			shader.set_uniform<glm::mat4>("model", model);
			shader.set_uniform<glm::mat4>("projection", projection);

//			sphere->integrate(1 / 60.f);

			sphere->draw();


			//model = glm::scale(glm::mat4(1.f), glm::vec3(0.5, 0.5, 0.5));
			//model = glm::translate(model, sphere1->getPosition()) * glm::mat4(sphere1->getRotation());
			//model = glm::scale(model, glm::vec3(sphere1->getRadius(), sphere1->getRadius(), sphere1->getRadius()));
			//
			//shader.set_uniform<glm::mat4>("view", view);
			//shader.set_uniform<glm::mat4>("model", model);
			//shader.set_uniform<glm::mat4>("projection", projection);

			//sphere1->integrate(1 / 60.f);

			//sphere1->draw();
			


			model = glm::scale(glm::mat4(1.f), glm::vec3(0.5, 0.5, 0.5));
			model = glm::translate(model, cube->getPosition()) * glm::mat4(cube->getRotation());
			model = glm::scale(model, cube->getExtents());
			shader.use();
			shader.set_uniform<glm::mat4>("view", view);
			shader.set_uniform<glm::mat4>("model", model);
			shader.set_uniform<glm::mat4>("projection", projection);
			
			cube->integrate(1 / 60.f);

			cube->draw();

			//Contact* c = new Contact();
			//if (sphere->isColliding(cube, c))
			//	handelCollision(c, 1.f);
			
			floorShader.use();
			floorShader.set_uniform<glm::mat4>("view", view);
			floorShader.set_uniform<glm::mat4>("projection", projection);
			tex.bind();
			Floor::getInstance()->draw();
			tex.unbind();

			cubeMap.draw();

			GLCall(glClearColor(clearColor.x, clearColor.y, clearColor.z, clearColor.w));

			ImGui::Render();
			ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

			glfwSwapBuffers(window);

			glfwPollEvents();

			processInput(window);
		}
	}

	ImGui::DestroyContext();
	glfwTerminate();

	return 0;
}