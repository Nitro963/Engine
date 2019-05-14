#include "vertexarray.h"
#include "basic.h"
namespace renderer {
	vertexarray::vertexarray() {
		GLCall(glGenVertexArrays(1, &id));
	}
	vertexarray::~vertexarray() {
		GLCall(glBindVertexArray(0));
		GLCall(glDeleteVertexArrays(1, &id));
	}
	void vertexarray::bind() const {
		GLCall(glBindVertexArray(id));
	}
	void vertexarray::unbind() const {
		GLCall(glBindVertexArray(0));
	}
	void vertexarray::addbuffer(const vertexbuffer& vb, const vertexbufferlayout& layout) {
		bind();
		vb.bind();
		const auto& elements = layout.getelements();
		unsigned int offset = 0;
		for (int i = 0; i < elements.size(); i++) {
			const auto& element = elements[i];
			GLCall(glEnableVertexAttribArray(i));
			GLCall(glVertexAttribPointer(i, element.count, element.type, element.normalized, layout.getstride(), (const void*)offset));
			offset += element.count * vertexbufferelement::getsize(element.type);
		}
		vb.unbind();
		unbind();
	}
}