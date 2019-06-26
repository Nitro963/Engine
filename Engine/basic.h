#pragma once
#include<GL/glew.h>
#include"glm\glm.hpp"
#define ASSERT(x) if (!(x)) __debugbreak();
#define GLCall(x) glClearError();\
	x;\
	ASSERT(glLogCall(#x, __FILE__, 	__LINE__))


void glClearError();

bool glLogCall(const char* function, const char* file, int line);

struct Material {
	float shininess;
	float epsilon;
	float mu;
};

struct DirLight {
	glm::vec3 direction;

	glm::vec3 ambient;
	glm::vec3 diffuse;
	glm::vec3 specular;
};

struct SpotLight {
	glm::vec3 position;
	glm::vec3 direction;
	float cutOff;
	float outerCutOff;

	float constant;
	float linear;
	float quadratic;

	glm::vec3 ambient;
	glm::vec3 diffuse;
	glm::vec3 specular;
};