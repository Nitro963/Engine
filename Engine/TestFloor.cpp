#include "TestFloor.h"

test::TestFloor::TestFloor() : point(0.5 ,0 ,0) ,force(0 ,9.8 ,0){
	cube = new SolidCuboid(1, glm::vec3(1, 1, 1));
	sphere = new SolidSphere(1, 0.5 ,glm::vec3(0.5 ,2 ,0.5));
	sphere1 = new SolidSphere(0.5, 0.5);
	std::vector<std::string> faces{
		"res/Textures/SkyBoxs/New folder/right.jpg",
		"res/textures/skyboxs/New folder/left.jpg",
		"res/textures/skyboxs/New folder/top.jpg",
		"res/textures/skyboxs/New folder/bottom.jpg",
		"res/textures/skyboxs/New folder/front.jpg",
		"res/textures/skyboxs/New folder/back.jpg"
	};
	cubeMap = new renderer::skybox(faces);
	mainShader = new renderer::shader("res/shaders/basic.shader");
}

test::TestFloor::~TestFloor(){
	delete cube;
	delete cubeMap;
	delete mainShader;
}

void test::TestFloor::OnRender(){
	GLCall(glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT));
	GLCall(glClearColor(0.f, 0.f, 0.f, 0.f));
	cubeMap->draw();
	OBB BV1(cube->getPosition(), cube->getRotationT(), cube->getExtents() * 0.5f);
	boundingSphere BV2(sphere->getPosition() ,sphere->getRadius());
	CollisionManifold manifold = BV2.findCollisionFeatures(BV1);

	sphere->integrate(1 / 60.f);
	cube->integrate(1 / 60.f);
	glm::mat4 view = camera.get_view_matrix();
	glm::mat4 projection = glm::perspective(glm::radians(camera.get_zoom()), (float)SCR_WIDTH / SCR_HEIGHT, 0.1f, 100.f);
	glm::mat4 model = glm::scale(glm::mat4(1.f), glm::vec3(0.5f, 0.5f, 0.5f));
	model = glm::translate(model, cube->getPosition()) * glm::mat4(cube->getRotation());
	model = glm::scale(model, cube->getExtents());
	mainShader->use();
	mainShader->set_uniform<glm::mat4>("view", view);
	mainShader->set_uniform<glm::mat4>("projection", projection);
	mainShader->set_uniform<glm::mat4>("model", model);
	cube->render();

	model = glm::scale(glm::mat4(1.f), glm::vec3(0.5f, 0.5f, 0.5f));
	model = glm::translate(model, sphere->getPosition()) * glm::mat4(sphere->getRotation());
	model = glm::scale(model, glm::vec3(sphere->getRadius()));
	mainShader->set_uniform<glm::mat4>("view", view);
	mainShader->set_uniform<glm::mat4>("projection", projection);
	mainShader->set_uniform<glm::mat4>("model", model);
	sphere->render();
}

void test::TestFloor::OnImGuiRender(){
	ImGui::InputFloat3("force", &force[0]);
	ImGui::InputFloat3("point", &point[0]);
	if (ImGui::Button("apply Force"))
		sphere->applyForce(point, force);
	if (ImGui::Button("negate Force"))
		sphere->applyForce(point, -force);
}
