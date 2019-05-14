#ifndef SHADER_H
#define SHADER_H
#pragma once

#include <string>
#include <glm\glm.hpp>
#include <glm\gtc\matrix_transform.hpp>
#include <glm\gtc\type_ptr.hpp>
#include <unordered_map>
#include "basic.h"
namespace renderer {

	struct shadersource {
		std::string vertexShader;
		std::string fragmentShader;
	};

	class shader {
	private:
		std::string filepath;
		unsigned int id;
		std::unordered_map<std::string, int> mp;
	public:
		shader(const std::string& filepath);
		~shader();

		void use()const;
		//void use_default() const;
		template<typename T>
		void set_uniform(const std::string& name, T& value);

		template<typename U>
		void set_uniform_array(const std::string& name, const U* data, int count);
		unsigned int get_id() { return id; }
	private:
		shadersource parseshader(const std::string& filePath);
		unsigned int createshader(const std::string& vertexShader, const std::string& fragmentShader);
		unsigned int compileshader(unsigned int type, const std::string& source);
		int getuniformlocation(const std::string& name);
	};
}
#endif

