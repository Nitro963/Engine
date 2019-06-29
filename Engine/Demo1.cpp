#include "Demo1.h"

test::Demo1::Demo1() {
	camera = renderer::camera(glm::vec3(7.f,6.f,7.f), glm::vec3(0.f, 1.f, 0.f), -135.f, -30.f);
	tree = new OcTree(glm::vec3(0.f), 32);// OcTree::buildTree(glm::vec3(0), 32);
	init();
	mainShader = new renderer::shader("res/shaders/basic.shader");
	update = 1;
}

void test::Demo1::init() {
	bodies.push_back(new SolidCuboid(1e12, glm::vec3(32, 4, 32), Material(), glm::vec3(0 ,-4 ,0)));
	tree->insert(bodies.back());
}

void test::Demo1::reset() {
	update = 1;
	tree->clear();
	registry.clear();
	for (auto& body : bodies)
		delete body;
	bodies.clear();
	init();
}

test::Demo1::~Demo1() {
	delete tree;
	registry.clear();
	for (auto& body : bodies)
		delete body;
	bodies.clear();
	delete mainShader;
}

void test::Demo1::OnRender() {
	if (debugRender) {
		GLCall(glPolygonMode(GL_FRONT_AND_BACK, GL_LINE));
	}
	else {
		GLCall(glPolygonMode(GL_FRONT_AND_BACK, GL_FILL));
	}
	GLCall(glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT));
	GLCall(glClearColor(0.f, 0.f, 0.f, 0.f));
	glm::mat4 view = camera.getViewMatrix();
	glm::mat4 projection = glm::perspective(glm::radians(camera.getZoom()), (float)SCR_WIDTH / SCR_HEIGHT, 0.1f, 100.f);
	mainShader->use();
	mainShader->set_uniform<glm::mat4>("view", view);
	mainShader->set_uniform<glm::mat4>("projection", projection);
	glm::mat4 model(1.f);
	model = glm::scale(model, glm::vec3(0.5));
	mainShader->set_uniform<glm::mat4>("model", model);
	//mask active branches and update nodes life span
	tree->dfs();
	//dfs on the tree for contacts
	std::vector<ContactData*> contacts = tree->getContacts(std::list<RigidBody*>());
	for (const auto& contact : contacts) {
		if (debugRender) {
			renderer::vertexbuffer VBO(&contact->M->contacts[0], contact->M->contacts.size() * sizeof(float) * 3);
			renderer::vertexarray VAO;
			renderer::vertexbufferlayout layout;
			layout.push<float>(3);
			VAO.addbuffer(VBO, layout);
			glPointSize(8);
			VAO.bind();
			GLCall(glDrawArrays(GL_POINTS, 0, contact->M->contacts.size()));
		}
		for (int i = 0; i < 8; i++)
			applyImpulse(contact);
	}

	for (auto& contact : contacts)
		resolveInterpentration(contact);

	if (update) {
		registry.remove_if(isDead);
		bodies.remove_if(isDead);
		registry.updateForces(1.f / ImGui::GetIO().Framerate);
		//integrate bodies, update the tree and
		//prune out any dead branches
		tree->update(1.f / ImGui::GetIO().Framerate);
	}

	for (const auto & b : bodies) {
		model = b->getModel(glm::vec3(0.5f));
		mainShader->set_uniform<glm::mat4>("model", model);
		b->render();
	}
}

void test::Demo1::OnImGuiRender() {
	ImGui::SameLine();
	if (ImGui::Button("Reset demo"))
		reset();
	ImGui::SameLine();
	ImGui::Checkbox("Update", &update);
	ImGui::SameLine();
	ImGui::Checkbox("debug render" ,&debugRender);
	ImGui::PushItemWidth(55);
	ImGui::InputFloat("Mass", &mass);
	ImGui::SameLine(); ImGui::InputFloat("Radius", &rad);
	ImGui::PopItemWidth();
	ImGui::InputFloat3("position", &pos[0]);
	if (ImGui::Button("add Sphere")) {
		bodies.push_back(new SolidSphere(mass, rad, Material(), pos));
		tree->insert(bodies.back());
		registry.add(bodies.back(), new GravityForce(glm::vec3(0, -9.8, 0)));
	}
	ImGui::InputFloat3("Cuboid Extents", &extents[0]);
	if (ImGui::Button("add Cuboid")) {
		bodies.push_back(new SolidCuboid(mass, extents, Material(), pos));
		tree->insert(bodies.back());
		registry.add(bodies.back(), new GravityForce(glm::vec3(0, -9.8, 0)));
	}
}
