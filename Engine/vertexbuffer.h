#ifndef VERTEX_BUFFER_H
#define VERTEX_BUFFER_H
#pragma once
namespace renderer {
	class vertexbuffer {
	private:
		unsigned int id;
	public:
		vertexbuffer(const void* data, unsigned int size);
		~vertexbuffer();
		void bind() const;
		void unbind() const;
	};
}
#endif