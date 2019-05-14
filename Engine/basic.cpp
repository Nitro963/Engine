#ifndef BASIC_H
#define BASIC_H
#include"basic.h"
#include<iostream>

void glclearerror() {
	while (glGetError());
}
bool gllogcall(const char* function, const char* file, int line) {
	while (GLenum error = glGetError()) {
		std::cout << "[OpenGl Error] (" << error << ") " << function << " " << file << " " << line << "\n";
		return false;
	}
	return true;
}
#endif