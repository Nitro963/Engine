#ifndef TEXTURE_H
#define TEXTURE_H
#pragma once
#include <string>
#include "basic.h"
#include "vendor\stb_image\stb_image.h"
namespace renderer {
	class texture {
	private:
		unsigned int id;
		std::string file_path;
		unsigned char* local_buffer;
		int width, height, bpp;
	public:
		texture(const std::string &path);
		~texture();
		void bind(unsigned int slot = 0) const;
		void unbind() const;

		inline int get_width() { return width; }
		inline int get_height() { return height; }
		inline std::string get_file_path() { return file_path; }
	};
}
#endif