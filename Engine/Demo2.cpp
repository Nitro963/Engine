#include "Demo2.h"

test::Demo2::Demo2() : update(true),picker(&modifyBody ,&read){
	camera = renderer::camera(glm::vec3(12.f), glm::vec3(0.f, 1.f, 0.f), -135.f, -30.f);
	tree = new OcTree(glm::vec3(0.f), 32);
	init();
	mainShader = new renderer::shader("res/shaders/basic.shader");
}

void test::Demo2::init(){
	bodies.push_back(new SolidCuboid(1e12, glm::vec3(16, 2, 16), Material(), glm::vec3(0, 0, 0)));
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

void test::Demo2::OnRender(){
	if (debugRender) {
		GLCall(glPolygonMode(GL_FRONT_AND_BACK, GL_LINE));
	}
	else {
		GLCall(glPolygonMode(GL_FRONT_AND_BACK, GL_FILL));
	}
	GLCall(glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT));
	GLCall(glClearColor(0.f, 0.f, 0.f, 0.f));
	picker.update(tree);
	glm::mat4 view = camera.getViewMatrix();
	glm::mat4 projection = glm::perspective(glm::radians(camera.getZoom()), (float)SCR_WIDTH / SCR_HEIGHT, 0.1f, 100.f);
	mainShader->use();
	mainShader->set_uniform<glm::mat4>("view", view);
	mainShader->set_uniform<glm::mat4>("projection", projection);
	glm::mat4 model(1.f);
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
			resolveContact(contact);
	}

	for (auto& contact : contacts)
		resolveInterpentration(contact);

	if (update) {
		float duration = 1.f / ImGui::GetIO().Framerate;
		registry.remove_if(isDead);
		registry.updateForces(duration);
		//integrate bodies, update the tree and
		//prune out any dead branches
		tree->update(duration);
	}
	for (auto it = bodies.begin(); it != bodies.end();) {
		auto body = *it;
		if (body->isDead()){
			auto tmp = it;
			++it;
			bodies.erase(tmp);
			if (modifyBody == body)
				modifyBody = nullptr;
			delete body;
			continue;
		}
		model = body->getModel(glm::vec3(1.f));
		mainShader->set_uniform<glm::mat4>("model", model);
		body->render();
		++it;
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
	ImGui::Checkbox("Integrate", &update);
	ImGui::SameLine();
	ImGui::Checkbox("Debug render", &debugRender);
	ImGui::Separator();
	if (modifyBody) {
		if (read) {
			sphere = nullptr;
			cuboid = nullptr;
			mass = modifyBody->getMass();
			sphere = dynamic_cast<SolidSphere*>(modifyBody);
			if (sphere)
				rad = sphere->getRadius();
			else {
				cuboid = dynamic_cast<SolidCuboid*>(modifyBody);
				extents = cuboid->getExtents();
			}
			read = false;
		}
	}
	ImGui::Text("Rigid body mass, position and orientation");
	ImGui::PushItemWidth(55);
	ImGui::InputFloat("Mass", &mass);
	ImGui::PopItemWidth();
	ImGui::PushItemWidth(200);
	ImGui::InputFloat3("Position", &pos[0]);
	ImGui::InputFloat3("Axis", &axis[0]);
	ImGui::SliderAngle("Angel", &theta);
	ImGui::PopItemWidth();
	ImGui::Separator();
	ImGui::Text("Solid Sphere");
	ImGui::PushItemWidth(55);
	ImGui::InputFloat("Radius", &rad);
	ImGui::PopItemWidth();
	if (ImGui::Button("add sphere")) {
		bodies.push_back(new SolidSphere(mass, rad, Material(), pos, glm::angleAxis(theta, axis)));
		tree->insert(bodies.back());
		registry.add(bodies.back(), new GravityForce(glm::vec3(0, -9.8, 0)));
	}
	ImGui::Separator();
	ImGui::PushItemWidth(200);
	ImGui::Text("Solid Cubiod");
	ImGui::InputFloat3("Cuboid Extents", &extents[0]);
	ImGui::PopItemWidth();
	if (ImGui::Button("Add cuboid")) {
		bodies.push_back(new SolidCuboid(mass, extents, Material(), pos, glm::angleAxis(theta, axis)));
		tree->insert(bodies.back());
		registry.add(bodies.back(), new GravityForce(glm::vec3(0, -9.8, 0)));
	}
	if (modifyBody) {
		ImGui::Separator();
		ImGui::Text("Add motor joint to the rigid body");
		ImGui::PushItemWidth(200);
		ImGui::InputFloat3("Force", &force[0]);
		ImGui::InputFloat3("Attatched point", &pt[0]);
		ImGui::PopItemWidth();
		ImGui::PushItemWidth(55);
		ImGui::InputFloat("Time", &t);
		ImGui::PopItemWidth();
		if (ImGui::Button("Add Motor"))
			registry.add(modifyBody, new MotorJoint(force, pt));
		ImGui::SameLine();
		if (ImGui::Button("Add Timed Motor"))
			registry.add(modifyBody, new TimedMotorJoint(t, force, pt));
		ImGui::SameLine();
		if (ImGui::Button("Apply force Once"))
			modifyBody->applyImpulse(pt, force);
		ImGui::Separator();
		ImGui::PushItemWidth(200);
		if (ImGui::Button("Apply modification")) {
			if (cuboid)
				cuboid->update(mass, extents);
			else
				sphere->update(mass, rad);
		}
	}
	ImGui::Text("Camera position:(%.3f, %.3f, %.3f)", camera.getPosition().x, camera.getPosition().y, camera.getPosition().z);
}