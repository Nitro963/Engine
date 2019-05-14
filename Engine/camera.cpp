#include "camera.h"
namespace renderer {
	camera::camera(glm::vec3 position, glm::vec3 up, float yaw, float pitch) : Front(glm::vec3(0.0f, 0.0f, -1.0f)), MovementSpeed(SPEED), MouseSensitivity(SENSITIVITY), Zoom(ZOOM) {
		this->position = position;
		this->world_up = up;
		this->yaw = yaw;
		this->pitch = pitch;
		update_camera_vectors();
	}

	camera::camera(float posX, float posY, float posZ, float upX, float upY, float upZ, float yaw, float pitch) : Front(glm::vec3(0.0f, 0.0f, -1.0f)), MovementSpeed(SPEED), MouseSensitivity(SENSITIVITY), Zoom(ZOOM) {
		position = glm::vec3(posX, posY, posZ);
		this->world_up = glm::vec3(upX, upY, upZ);
		this->yaw = yaw;
		this->pitch = pitch;
		update_camera_vectors();
	}
	glm::mat4 camera::get_view_matrix() {
		return glm::lookAt(position, position + Front, Up);
	}

	void camera::process_keyboard(Camera_Movement direction, float deltaTime) {
		float velocity = MovementSpeed * deltaTime;
		if (direction == FORWARD)
			position += Front * velocity;
		if (direction == BACKWARD)
			position -= Front * velocity;
		if (direction == LEFT)
			position -= Right * velocity;
		if (direction == RIGHT)
			position += Right * velocity;
		//position.y = 0.0f;
	}


	void camera::process_mouse_movement(float xoffset, float yoffset, bool constrainPitch) {
		xoffset *= MouseSensitivity;
		yoffset *= MouseSensitivity;

		yaw += xoffset;
		pitch += yoffset;

		// Make sure that when pitch is out of bounds, screen doesn't get flipped
		if (constrainPitch) {
			if (pitch > 89.0f)
				pitch = 89.0f;
			if (pitch < -89.0f)
				pitch = -89.0f;
		}

		// Update Front, Right and Up Vectors using the updated Euler angles
		update_camera_vectors();
	}

	void camera::process_mouse_scroll(float yoffset) {
		if (Zoom >= 1.0f && Zoom <= 45.f)
			Zoom -= yoffset;
		if (Zoom <= 1.0f)
			Zoom = 1.0f;
		if (Zoom >= 45.f)
			Zoom = 45.0f;
	}

	void camera::update_camera_vectors() {
		// Calculate the new Front vector
		glm::vec3 front;
		front.x = cos(glm::radians(yaw)) * cos(glm::radians(pitch));
		front.y = sin(glm::radians(pitch));
		front.z = sin(glm::radians(yaw)) * cos(glm::radians(pitch));
		Front = glm::normalize(front);
		// Also re-calculate the Right and Up vector
		Right = glm::normalize(glm::cross(Front, world_up));  // Normalize the vectors, because their length gets closer to 0 the more you look up or down which results in slower movement.
		Up = glm::normalize(glm::cross(Right, Front));
	}
}