#include "texture.h"
#include <iostream>
namespace renderer {
	texture::texture(const std::string & path) : id(0), file_path(path), local_buffer(nullptr), width(0), height(0), bpp(0) {
		stbi_set_flip_vertically_on_load(1);
		local_buffer = stbi_load(path.c_str(), &width, &height, &bpp, 4);
		if (!local_buffer) {
			std::cout << "ERROR::texture: " << "failed to load image from file! " << file_path << '\n';
			return;
		}
		std::cout << "texture loaded at path: " << path << '\n';
		GLCall(glGenTextures(1, &id));
		GLCall(glBindTexture(GL_TEXTURE_2D, id));
		GLCall(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST));
		GLCall(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST));
		GLCall(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE));
		GLCall(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE));
		GLCall(glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, local_buffer));
		GLCall(glGenerateMipmap(GL_TEXTURE_2D));
		GLCall(glBindTexture(GL_TEXTURE_2D, 0));
		if (local_buffer)
			stbi_image_free(local_buffer);
	}

	texture::~texture() {
		GLCall(glDeleteTextures(1, &id));
	}

	void texture::bind(unsigned int slot) const {
		GLCall(glActiveTexture(GL_TEXTURE0 + slot));
		GLCall(glBindTexture(GL_TEXTURE_2D, id));

	}

	void texture::unbind() const {
		GLCall(glBindTexture(GL_TEXTURE_2D, 0));
	}
}