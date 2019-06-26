#include "Demo2.h"

test::Demo2::Demo2() : update(true){
	tree = new OcTree(glm::vec3(0.f), 32);
	init();
	mainShader = new renderer::shader("res/shaders/basic.shader");
}

void test::Demo2::init(){
	bodies.push_back(new SolidCuboid(1e12, glm::vec3(1, 1, 1), glm::vec3(0, 0, 0)));
	//bodies.push_back(new SolidSphere(1e12, 0.5));
	tree->insert(bodies.back());
}

void test::Demo2::reset(){
	update = 1;
	tree->clear();
	registry.clear();
	for (auto& body : bodies)
		delete body;
	bodies.clear();
	init();

}

test::Demo2::~Demo2(){
	delete tree;
	registry.clear();
	for (auto& body : bodies)
		delete body;
	bodies.clear();
	delete mainShader;
}

bool isDead1(RigidBody*& body) {
	return body->isDead();
}

void test::Demo2::OnRender(){
	if (debugRender) {
		GLCall(glPolygonMode(GL_FRONT_AND_BACK, GL_LINE));
	}
	else {
		GLCall(glPolygonMode(GL_FRONT_AND_BACK, GL_FILL));
	}
	GLCall(glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT));
	GLCall(glClearColor(0.f, 0.f, 0.f, 0.f));
	picker.update();
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
			applyImpulse(contact, 0.6);
	}

	for (auto& contact : contacts)
		resolveInterpentration(contact);

	if (update) {
		registry.remove_if(isDead1);
		bodies.remove_if(isDead1);
		registry.updateForces();
		//integrate bodies, update the tree and
		//prune out any dead branches
		tree->update(1.f / ImGui::GetIO().Framerate);
	}
	for (const auto & b : bodies) {
		model = b->getModel(glm::vec3(1.f));
		mainShader->set_uniform<glm::mat4>("model", model);
		b->render();
	}
}

void test::Demo2::OnImGuiRender(){
	ImGui::SameLine();
	if (ImGui::Button("Reset demo"))
		reset();
	ImGui::SameLine();
	ImGui::Checkbox("update", &update);
	ImGui::SameLine();
	ImGui::Checkbox("debug render", &debugRender);
	ImGui::Separator();
	ImGui::Text("RigidBody mass, position and orientation");
	ImGui::PushItemWidth(55);
	ImGui::InputFloat("Mass", &mass);
	ImGui::PopItemWidth();
	ImGui::PushItemWidth(200);
	ImGui::InputFloat3("position", &pos[0]);
	ImGui::InputFloat3("axis", &axis[0]);
	ImGui::SliderAngle("Angel", &theta);
	ImGui::PopItemWidth();
	ImGui::Separator();
	ImGui::Text("Solid Sphere");
	ImGui::PushItemWidth(55);
	ImGui::InputFloat("Radius", &rad);
	ImGui::PopItemWidth();
	if (ImGui::Button("add Sphere")) {
		bodies.push_back(new SolidSphere(mass, rad, pos, glm::angleAxis(theta, axis)));
		tree->insert(bodies.back());
		registry.add(bodies.back(), new GravityForce(glm::vec3(0, -9.8, 0)));
	}
	ImGui::Separator();
	ImGui::PushItemWidth(200);
	ImGui::Text("Solid Cubiod");
	ImGui::InputFloat3("Cuboid Extents", &extents[0]);
	ImGui::PopItemWidth();
	if (ImGui::Button("add Cuboid")) {
		bodies.push_back(new SolidCuboid(mass, extents, pos, glm::angleAxis(theta, axis)));
		tree->insert(bodies.back());
		registry.add(bodies.back(), new GravityForce(glm::vec3(0, -9.8, 0)));
	}
	ImGui::Separator();
	ImGui::Text("Motor joint to the last Rigid body");
	ImGui::PushItemWidth(200);
	ImGui::InputFloat3("force", &force[0]);
	ImGui::InputFloat3("attatched point", &pt[0]);
	ImGui::PopItemWidth();
	if (ImGui::Button("Add Motor")) {
		registry.add(bodies.back(), new MotorJoint(force, pos));
	}
}
