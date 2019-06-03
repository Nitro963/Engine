#include "TestQubes.h"

test::Demo::Demo():point(0.5, 0, 0), force(0, 9.8, 0) {
	bodies.push_back(new SolidCuboid(1e9, glm::vec3(10, 0.2, 20)));
	bodies.push_back(new SolidCuboid(1, glm::vec3(1, 1, 1) ,glm::vec3(0 ,2 ,0)));
	bodies.push_back(new SolidSphere(1, 0.5, glm::vec3(1, 1.5, 0)));
	mainShader = new renderer::shader("res/shaders/basic.shader");
}

test::Demo::~Demo(){
	for (int i = 0; i < bodies.size(); i++)
		delete bodies[i];
	delete mainShader;
}

void test::Demo::OnRender(){
	GLCall(glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT));
	GLCall(glClearColor(0.f, 0.f, 0.f, 0.f));
	std::vector<Contact*> ve = searchForContacts(bodies);
	for (auto& contact : ve)
		applyImpulse(contact, 0.6);
	for (auto& b : bodies)
		b->integrate(1 / 60.f);
	for (auto& contact : ve)
		resolveInterpentration(contact);

	glm::mat4 view = camera.get_view_matrix();
	glm::mat4 projection = glm::perspective(glm::radians(camera.get_zoom()), (float)SCR_WIDTH / SCR_HEIGHT, 0.1f, 100.f);
	mainShader->use();
	mainShader->set_uniform<glm::mat4>("view", view);
	mainShader->set_uniform<glm::mat4>("projection", projection);

	for (auto const & b : bodies) {
		glm::mat4 model = b->getModel(glm::vec3(0.5));
		mainShader->set_uniform<glm::mat4>("model", model);
		b->render();
	}
}

void test::Demo::OnImGuiRender(){
	ImGui::InputFloat3("force", &force[0]);
	ImGui::InputFloat3("point", &point[0]);
	if (ImGui::Button("apply Force cube"))
		bodies[0]->applyForce(point, force);
	if (ImGui::Button("negate Force cube"))
		bodies[0]->applyForce(point, -force);
	if (ImGui::Button("apply Force cube1"))
		bodies[1]->applyForce(point, force);
	if (ImGui::Button("negate Force cube1"))
		bodies[1]->applyForce(point, -force);
	if (ImGui::Button("apply Force sphere"))
		bodies[2]->applyForce(point, force);
	if (ImGui::Button("negate Force sphere"))
		bodies[2]->applyForce(point, -force);
}
