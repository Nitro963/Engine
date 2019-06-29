#include "Demo3.h"

const char* test::Demo3::gravity[4] = { "Earth","Moon","Saturn","Jupiter" };

const char* test::Demo3::currentgravity = test::Demo3::gravity[0];

test::Demo3::Demo3() : picker(&modifyBody, &read) {
	camera = renderer::camera(glm::vec3(12.f), glm::vec3(0.f, 1.f, 0.f), -135.f, -30.f);
	tree = new OcTree(glm::vec3(0.f), 32);
	init();
	mainShader = new renderer::shader("res/shaders/basic.shader");
}

void test::Demo3::init() {
	modifyBody = nullptr;
	bodies.push_back(new SolidCuboid(1e12, glm::vec3(16, 2, 16), Material(), glm::vec3(0, 0, 0)));
	tree->insert(bodies.back());
	currentgravity = gravity[0];
}

void test::Demo3::reset() {
	update = 1;
	tree->clear();
	registry.clear();
	Jregistry.clear();
	for (auto& body : bodies)
		delete body;
	bodies.clear();
	init();
}

test::Demo3::~Demo3() {
	delete tree;
	registry.clear();
	Jregistry.clear();
	for (auto& body : bodies)
		delete body;
	bodies.clear();
	delete mainShader;
	delete Earth;
	delete Moon;
	delete Saturn;
	delete Jupiter;
}

void test::Demo3::OnRender() {
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
			applyImpulse(contact);
	}

	for (auto& contact : contacts)
		resolveInterpentration(contact);

	if (update) {
		float duration = 1.f / ImGui::GetIO().Framerate;
		//remove dead bodies
		registry.remove_if(isDead);
		//updage forces for all bodies
		registry.updateForces(duration);
		//integrate bodies, update the tree and
		//prune out any dead branches
		tree->update(duration);
		//solve constraints of all bodies
		Jregistry.solveConstraints(duration);
	}
	for (auto it = bodies.begin(); it != bodies.end();) {
		auto body = *it;
		if (body->isDead()) {
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

void test::Demo3::OnImGuiRender() {
	if (modifyBody) {
		if (read) {
			cuboid = nullptr;
			sphere = nullptr;
			bodyMass = modifyBody->getMass();
			bodyPos = modifyBody->getPosition();
			bodyMaterial = modifyBody->getMaterial();
			sphere = dynamic_cast<SolidSphere*>(modifyBody);
			if (sphere)
				bodyRad = sphere->getRadius();
			else {
				cuboid = dynamic_cast<SolidCuboid*>(modifyBody);
				bodyExtents = cuboid->getExtents();
			}
			read = false;
		}
	}
	ImGui::SameLine();
	if (ImGui::Button("Reset demo"))
		reset();
	ImGui::SameLine();
	ImGui::Checkbox("Integrate", &update);
	ImGui::SameLine();
	ImGui::Checkbox("Debug render", &debugRender);
	ImGui::Separator();
	ImGui::PushItemWidth(70);
	if (ImGui::BeginCombo("Gravity", currentgravity)) {
		for (int i = 0; i < 4; i++) {
			bool isSelected = currentgravity == gravity[i];
			if (ImGui::Selectable(gravity[i], isSelected))
				currentgravity = gravity[i];
			if (isSelected)
				ImGui::SetItemDefaultFocus();
		}
		ImGui::EndCombo();
	}
	ImGui::PopItemWidth();
	ImGui::Separator();
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
		if (currentgravity == gravity[0])
			registry.add(bodies.back(), Earth);
		if (currentgravity == gravity[1])
			registry.add(bodies.back(), Moon);
		if (currentgravity == gravity[2])
			registry.add(bodies.back(), Saturn);
		if (currentgravity == gravity[3])
			registry.add(bodies.back(), Jupiter);
	}
	ImGui::Separator();
	ImGui::PushItemWidth(200);
	ImGui::Text("Solid Cubiod");
	ImGui::InputFloat3("Cuboid Extents", &extents[0]);
	ImGui::PopItemWidth();
	if (ImGui::Button("Add cuboid")) {
		bodies.push_back(new SolidCuboid(mass, extents, Material(), pos, glm::angleAxis(theta, axis)));
		tree->insert(bodies.back());
		if (currentgravity == gravity[0])
			registry.add(bodies.back(), Earth);
		if (currentgravity == gravity[1])
			registry.add(bodies.back(), Moon);
		if (currentgravity == gravity[2])
			registry.add(bodies.back(), Saturn);
		if (currentgravity == gravity[3])
			registry.add(bodies.back(), Jupiter);
	}
	if (modifyBody) {
		ImGui::Begin("Modify body");
		ImGui::Text("Rigid body mass ,Material and position");
		
		ImGui::PushItemWidth(55);
		ImGui::InputFloat("Mass", &bodyMass);
		ImGui::InputFloat("Coefficient of restitution", &bodyMaterial.epsilon);
		ImGui::InputFloat("Coefficient of static friction", &bodyMaterial.mu);
		ImGui::InputFloat("Coefficient of dynamic friction", &bodyMaterial.muDynamic);
		ImGui::PopItemWidth();
		ImGui::PushItemWidth(200);
		ImGui::InputFloat3("Position", &bodyPos[0]);
		ImGui::PopItemWidth();
		ImGui::Separator();
		if (sphere) {
			ImGui::Text("Solid Sphere");
			ImGui::PushItemWidth(55);
			ImGui::InputFloat("Radius", &bodyRad);
			ImGui::PopItemWidth();
			ImGui::Separator();
		}
		else {
			ImGui::PushItemWidth(200);
			ImGui::Text("Solid Cubiod");
			ImGui::InputFloat3("Cuboid Extents", &bodyExtents[0]);
			ImGui::PopItemWidth();
			ImGui::Separator();
		}
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
			modifyBody->applyForce(pt, force);
		ImGui::Separator();
		ImGui::Text("Add a fixed joint");
		ImGui::PushItemWidth(200);
		ImGui::InputFloat3("Fixed at point", &Jpt[0]);
		ImGui::PopItemWidth();
		if (ImGui::Button("Add joint"))
			Jregistry.add(modifyBody, new FixedJoint(Jpt));
		ImGui::Separator();
		ImGui::PushItemWidth(200);
		if (ImGui::Button("Apply modification")) {
			if (cuboid)
				cuboid->update(bodyMass, bodyExtents);
			else
				sphere->update(bodyMass, bodyRad);
			modifyBody->setPosition(bodyPos);
			modifyBody->setMaterial(bodyMaterial);
		}
		ImGui::SameLine();
		if (ImGui::Button("Delete body")) {
			registry.remove(modifyBody);
			Jregistry.remove(modifyBody);
			bodies.remove(modifyBody);
			tree->remove(modifyBody);
			delete modifyBody;
			modifyBody = nullptr;
		}
		ImGui::End();
	}
	ImGui::Text("Camera position:(%.3f, %.3f, %.3f)", camera.getPosition().x, camera.getPosition().y, camera.getPosition().z);
}