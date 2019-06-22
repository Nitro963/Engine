#include "Demo1.h"

test::Demo1::Demo1() {
	tree = new OcTree(glm::vec3(0), 32);
	init();
	mainShader = new renderer::shader("res/shaders/basic.shader");
}

void test::Demo1::init() {
	bodies.push_back(new SolidCuboid(1e12, glm::vec3(20, 0.2, 20), glm::vec3()));
	tree->insert(bodies.back());
}

void test::Demo1::reset() {
	update = 0;
	registry.clear();
	tree->clear();
	for (auto& body : bodies)
		delete body;
	bodies.clear();
	init();
}

test::Demo1::~Demo1() {
	registry.clear();
	for (auto& body : bodies)
		delete body;
	bodies.clear();
	delete mainShader;
}

bool isDead(RigidBody*& body){
	return body->isDead();
}

void test::Demo1::OnRender() {
	if (debugRender) {
		GLCall(glPolygonMode(GL_FRONT_AND_BACK, GL_LINE));
	}
	else {
		GLCall(glPolygonMode(GL_FRONT_AND_BACK, GL_FILL));
	}
	GLCall(glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT));
	GLCall(glClearColor(0.f, 0.f, 1.f, 0.f));
	glm::mat4 view = camera.get_view_matrix();
	glm::mat4 projection = glm::perspective(glm::radians(camera.get_zoom()), (float)SCR_WIDTH / SCR_HEIGHT, 0.1f, 100.f);
	mainShader->use();
	mainShader->set_uniform<glm::mat4>("view", view);
	mainShader->set_uniform<glm::mat4>("projection", projection);
	glm::mat4 model(1.f);
	model = glm::scale(model, glm::vec3(0.5));
	mainShader->set_uniform<glm::mat4>("model", model);
	std::vector<ContactData*> contacts = tree->getContacts(std::list<RigidBody*>());
	for (const auto& contact : contacts) {
		if (debugRender) {
			renderer::vertexbuffer VBO1(&contact->M->contacts[0], contact->M->contacts.size() * sizeof(float) * 3);
			renderer::vertexarray VAO1;
			renderer::vertexbufferlayout layout;
			layout.push<float>(3);
			VAO1.addbuffer(VBO1, layout);
			glPointSize(5);
			VAO1.bind();
			GLCall(glDrawArrays(GL_POINTS, 0, contact->M->contacts.size()));
		}
		for (int i = 0; i < 8; i++)
			applyImpulse(contact, 0.6);
	}

	for (auto& contact : contacts)
		resolveInterpentration(contact);

	if (update) {
		registry.remove_if(isDead);
		bodies.remove_if(isDead);
		registry.updateForces(1 / 60.f);
		tree->update(1 / 60.f);
	}

	for (const auto & b : bodies) {
		model = b->getModel(glm::vec3(0.5));
		mainShader->set_uniform<glm::mat4>("model", model);
		b->render();
	}
}

void test::Demo1::OnImGuiRender() {
	ImGui::SameLine();
	if (ImGui::Button("Reset demo"))
		reset();
	ImGui::SameLine();
	if (ImGui::Button("update"))
		update ^= 1;
	ImGui::SameLine();
	ImGui::Checkbox("debug render" ,&debugRender);
	ImGui::PushItemWidth(55);
	ImGui::InputFloat("Mass", &mass);
	ImGui::SameLine(); ImGui::InputFloat("Radius", &rad);
	ImGui::PopItemWidth();
	ImGui::InputFloat3("position", &pos[0]);
	if (ImGui::Button("add Sphere")) {
		bodies.push_back(new SolidSphere(mass, rad, pos));
		tree->insert(bodies.back());
		registry.add(bodies.back(), new GravityForce(glm::vec3(0, -9.8, 0)));
	}
	ImGui::InputFloat3("Cuboid Extents", &extents[0]);
	if (ImGui::Button("add Cuboid")) {
		bodies.push_back(new SolidCuboid(mass, extents, pos));
		tree->insert(bodies.back());
		registry.add(bodies.back(), new GravityForce(glm::vec3(0, -9.8, 0)));
	}
}
