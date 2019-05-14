#include "vertexbuffer.h"
#include "basic.h"
#include <iostream>
namespace renderer {
	vertexbuffer::vertexbuffer(const void * data, unsigned int size) {
		GLCall(glGenBuffers(1, &id));
		GLCall(glBindBuffer(GL_ARRAY_BUFFER, id));
		GLCall(glBufferData(GL_ARRAY_BUFFER, size, data, GL_STATIC_DRAW));
	}

	vertexbuffer::~vertexbuffer() {
		GLCall(glBindBuffer(GL_ARRAY_BUFFER, 0));
		GLCall(glDeleteBuffers(1, &id));
	}


	void vertexbuffer::bind()const {
		GLCall(glBindBuffer(GL_ARRAY_BUFFER, id));
	}

	void vertexbuffer::unbind()const {
		GLCall(glBindBuffer(GL_ARRAY_BUFFER, 0));
	}
}