#include "MousePicker.h"

glm::vec4 MousePicker::toEyeCoords(glm::vec4 clipCoords){
	glm::mat4 invProjection = glm::inverse(glm::perspective(glm::radians(camera.getZoom()), (float)SCR_WIDTH / SCR_HEIGHT, 0.1f, 100.f));
	glm::vec4 eyeCoords = invProjection * clipCoords;
	return glm::vec4(eyeCoords.x, eyeCoords.y, -1, 0);
}

glm::vec3 MousePicker::toWorldCoord(glm::vec4 eyeCoords){
	return glm::vec3(glm::normalize(glm::inverse(camera.getViewMatrix()) * eyeCoords));
}

void MousePicker::update(OcTree* tree){
	if (ImGui::GetIO().MouseClicked[0] && ImGui::GetIO().MouseDownOwned && !ImGui::GetIO().WantCaptureMouse) {
		*modifyBody = nullptr;
		glm::vec2 normalizedCoords = getNormalizedDeviceCoords(glm::vec2(ImGui::GetIO().MousePos.x, ImGui::GetIO().MousePos.y));
		glm::vec4 clipCoords = glm::vec4(normalizedCoords.x, normalizedCoords.y, -1.f, 1.f);
		glm::vec4 eyeCoords = toEyeCoords(clipCoords);
		m_currentRay = Ray(camera.getPosition(), toWorldCoord(eyeCoords));
		float t = 1e9;
		tree->getNearest(m_currentRay, *modifyBody, t);
		*read = modifyBody;
	}
}
