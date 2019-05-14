#pragma once
#include<GL/glew.h>

#define ASSERT(x) if (!(x)) __debugbreak();
#define GLCall(x) glclearerror();\
	x;\
	ASSERT(gllogcall(#x ,__FILE__ ,	__LINE__))


void glclearerror();

bool gllogcall(const char* function ,const char* file ,int line);