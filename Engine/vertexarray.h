#ifndef VERTEX_ARRAY_H
#define VERTEX_ARRAY_H
#pragma once
#include "vertexbuffer.h"
#include "vertexbufferlayout.h"
#include <vector>
namespace renderer {
	class vertexarray {
	private:
		unsigned int id;
	public:
		vertexarray();
		~vertexarray();

		void addbuffer(const vertexbuffer& vb, const vertexbufferlayout& layout);

		void bind() const;
		void unbind() const;
	};
}
#endif