#include "indexbuffer.h"
#include "basic.h"
#include <iostream>
namespace renderer {
	indexbuffer::indexbuffer(const unsigned int *data, unsigned int count) :count(count) {
		ASSERT(sizeof(unsigned int) == sizeof(GLuint));
		GLCall(glGenBuffers(1, &id));
		GLCall(glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, id));
		GLCall(glBufferData(GL_ELEMENT_ARRAY_BUFFER, count * sizeof(unsigned int), data, GL_STATIC_DRAW));
	}

	indexbuffer::~indexbuffer() {
		GLCall(glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0));
		GLCall(glDeleteBuffers(GL_ELEMENT_ARRAY_BUFFER, &id));
	}

	void indexbuffer::bind() const {
		GLCall(glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, id));
	}

	void indexbuffer::unbind() const {
		GLCall(glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0));
	}
}
