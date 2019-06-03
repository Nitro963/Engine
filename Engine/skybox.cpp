#include "skybox.h"
#include <iostream>

namespace renderer {
	std::vector<float> skybox::vertices = {
		-1.0f,  1.0f, -1.0f,
		-1.0f, -1.0f, -1.0f,
		1.0f, -1.0f, -1.0f,
		1.0f, -1.0f, -1.0f,
		1.0f,  1.0f, -1.0f,
		-1.0f,  1.0f, -1.0f,

		-1.0f, -1.0f,  1.0f,
		-1.0f, -1.0f, -1.0f,
		-1.0f,  1.0f, -1.0f,
		-1.0f,  1.0f, -1.0f,
		-1.0f,  1.0f,  1.0f,
		-1.0f, -1.0f,  1.0f,

		1.0f, -1.0f, -1.0f,
		1.0f, -1.0f,  1.0f,
		1.0f,  1.0f,  1.0f,
		1.0f,  1.0f,  1.0f,
		1.0f,  1.0f, -1.0f,
		1.0f, -1.0f, -1.0f,

		-1.0f, -1.0f,  1.0f,
		-1.0f,  1.0f,  1.0f,
		1.0f,  1.0f,  1.0f,
		1.0f,  1.0f,  1.0f,
		1.0f, -1.0f,  1.0f,
		-1.0f, -1.0f,  1.0f,

		-1.0f,  1.0f, -1.0f,
		1.0f,  1.0f, -1.0f,
		1.0f,  1.0f,  1.0f,
		1.0f,  1.0f,  1.0f,
		-1.0f,  1.0f,  1.0f,
		-1.0f,  1.0f, -1.0f,

		-1.0f, -1.0f, -1.0f,
		-1.0f, -1.0f,  1.0f,
		1.0f, -1.0f, -1.0f,
		1.0f, -1.0f, -1.0f,
		-1.0f, -1.0f,  1.0f,
		1.0f, -1.0f,  1.0f 
	};
	vertexbuffer* skybox::VBO = nullptr;
	vertexarray* skybox::VAO = nullptr;
	shader* skybox::shad = nullptr;
	skybox::skybox(const std::vector<std::string>& faces){
		if (!VBO) {
			VBO = new renderer::vertexbuffer(&vertices[0], sizeof(float) * vertices.size());
			VAO = new renderer::vertexarray();
			vertexbufferlayout layout;
			layout.push<float>(3);
			VAO->addbuffer(*VBO, layout);
			shad = new shader("res/shaders/cubeMap.shader");
		}
		stbi_set_flip_vertically_on_load(0);
		GLCall(glGenTextures(1, &m_id));
		GLCall(glBindTexture(GL_TEXTURE_CUBE_MAP, m_id));
		int width, height, nrChannels;
		for (unsigned int i = 0; i < faces.size(); i++){
			
			unsigned char *data = stbi_load(faces[i].c_str(), &width, &height, &nrChannels, 0);
			if (data){
				std::cout << "Cubemap texture loaded at path: " << faces[i] << '\n';
				GLCall(glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X + i, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, data));
				stbi_image_free(data);
			}
			else{
				std::cout << "Cubemap texture failed to load at path: " << faces[i] << '\n';
				stbi_image_free(data);
			}
		}
		GLCall(glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_LINEAR));
		GLCall(glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_LINEAR));
		GLCall(glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE));
		GLCall(glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE));
		GLCall(glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE));
	}
	
	skybox::~skybox() {
		GLCall(glDeleteTextures(1, &m_id));
	}

	void skybox::draw() {
		glm::mat4 view = glm::mat4(glm::mat3(::camera.get_view_matrix()));
		glm::mat4 projection = glm::perspective(glm::radians(::camera.get_zoom()), (float)SCR_WIDTH / SCR_HEIGHT, 0.1f, 100.f);
		shad->use();
		shad->set_uniform<glm::mat4>("view", view);
		shad->set_uniform<glm::mat4>("projection", projection);
		GLCall(glDepthFunc(GL_LEQUAL));  // change depth function so depth test passes when values are equal to depth buffer's content
		VAO->bind();
		GLCall(glActiveTexture(GL_TEXTURE0));
		GLCall(glBindTexture(GL_TEXTURE_CUBE_MAP, m_id));
		GLCall(glDrawArrays(GL_TRIANGLES, 0, 36));
		VAO->unbind();
		GLCall(glDepthFunc(GL_LESS)); // set depth function back to default
	}
}
