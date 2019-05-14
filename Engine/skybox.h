#ifndef SKYBOX_H
#define SKYBOX_H
#include "basic.h"
#include "vertexbuffer.h"
#include "vertexarray.h"
#include "vertexbufferlayout.h"
#include "shader.h"
#include "camera.h"
#include "vendor\stb_image\stb_image.h"
#include <string>
#include <vector>

extern renderer::camera camera;

extern const unsigned int SCR_WIDTH;
extern const unsigned int SCR_HEIGHT;

namespace renderer {
	class skybox {
		private:
			static std::vector<float> vertices;
			static vertexbuffer* VBO;
			static vertexarray* VAO;
			static shader* shad;
			unsigned int m_id;
	public:
		skybox(const std::vector<std::string>& faces);
		~skybox();
		void draw();
	};
}
#endif // !SKYBOX_H
