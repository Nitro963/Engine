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
#include <memory>

extern renderer::camera camera;

extern const unsigned int SCR_WIDTH;
extern const unsigned int SCR_HEIGHT;

namespace renderer {
	class skybox {
	private:
		static std::vector<float> vertices;
		std::unique_ptr<vertexbuffer> m_VBO;
		std::unique_ptr<vertexarray> m_VAO;
		std::unique_ptr<shader> m_Shader;
		unsigned int m_id;
	public:
		skybox(const std::vector<std::string>& faces);
		~skybox();
		void draw();
	};
}
#endif // !SKYBOX_H
