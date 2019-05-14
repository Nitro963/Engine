#ifndef CAMERA_H
#define CAMERA_H

#include <glm\glm.hpp>
#include <glm\gtc\matrix_transform.hpp>

#include <vector>
namespace renderer {
	// Defines several possible options for camera movement. Used as abstraction to stay away from window-system specific input methods
	enum Camera_Movement {
		FORWARD,
		BACKWARD,
		LEFT,
		RIGHT
	};

	// Default camera values
	const float YAW = -90.0f;
	const float PITCH = 0.0f;
	const float SPEED = 2.5f;
	const float SENSITIVITY = 0.1f;
	const float ZOOM = 45.f;


	// An abstract camera class that processes input and calculates the corresponding Euler Angles, Vectors and Matrices for use in OpenGL
	class camera {
	private:
		// Camera Attributes
		glm::vec3 position;
		glm::vec3 Front;
		glm::vec3 Up;
		glm::vec3 Right;
		glm::vec3 world_up;
		// Euler Angles
		float yaw;
		float pitch;
		// Camera options
		float MovementSpeed;
		float MouseSensitivity;
		float Zoom;
	public:
		// Constructor with vectors
		camera(glm::vec3 position = glm::vec3(0.0f, 0.0f, 3.0f), glm::vec3 up = glm::vec3(0.0f, 1.0f, 0.0f), float yaw = YAW, float pitch = PITCH);
		// Constructor with scalar values
		camera(float posX, float posY, float posZ, float upX, float upY, float upZ, float yaw, float pitch);

		// Returns the view matrix calculated using Euler Angles and the LookAt Matrix
		glm::mat4 get_view_matrix();

		inline glm::vec3 get_position() { return position; }

		inline float get_zoom() { return Zoom; }

		inline glm::vec3 get_front() { return Front; }

		// Processes input received from any keyboard-like input system. Accepts input parameter in the form of camera defined ENUM (to abstract it from windowing systems)
		void process_keyboard(Camera_Movement direction, float deltaTime);

		// Processes input received from a mouse input system. Expects the offset value in both the x and y direction.
		void process_mouse_movement(float xoffset, float yoffset, bool constrainPitch = true);

		// Processes input received from a mouse scroll-wheel event. Only requires input on the vertical wheel-axis
		void process_mouse_scroll(float yoffset);

	private:
		// Calculates the front vector from the Camera's (updated) Euler Angles
		void update_camera_vectors();
	};
}
#endif