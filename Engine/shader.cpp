#include "shader.h"
#include<iostream>
#include<fstream>
#include<string>
#include<sstream>

#include"basic.h"
namespace renderer {
	shader::shader(const std::string & filepath) :filepath(filepath), id(0) {
		shadersource  src = parseshader(filepath);
		id = createshader(src.vertexShader, src.fragmentShader);
	}

	shader::~shader() {
		GLCall(glDeleteProgram(id));
	}

	void shader::use() const {
		GLCall(glUseProgram(id));
	}

	template<>
	void shader::set_uniform<int>(const std::string& name, int& value) {
		GLCall(glUniform1i(getuniformlocation(name), value));
	}

	template<>
	void shader::set_uniform<float>(const std::string& name, float& value) {
		GLCall(glUniform1f(getuniformlocation(name), value));
	}

	template<>
	void shader::set_uniform<glm::vec2>(const std::string& name, glm::vec2& vector) {
		GLCall(glUniform2fv(getuniformlocation(name), 1, glm::value_ptr(vector)));
	}

	template<>
	void shader::set_uniform<glm::vec3>(const std::string& name, glm::vec3& vector) {
		GLCall(glUniform3fv(getuniformlocation(name), 1, glm::value_ptr(vector)));
	}

	template<>
	void shader::set_uniform<glm::vec4>(const std::string& name, glm::vec4& vector) {
		GLCall(glUniform4fv(getuniformlocation(name), 1, glm::value_ptr(vector)));
	}

	template<>
	void shader::set_uniform<glm::mat4>(const std::string& name, glm::mat4& matrix) {
		GLCall(glUniformMatrix4fv(getuniformlocation(name), 1, GL_FALSE, glm::value_ptr(matrix)));
	}

	int shader::getuniformlocation(const std::string & name) {
		if (mp.find(name) != mp.end())
			return mp[name];
		GLCall(int location = glGetUniformLocation(id, name.c_str()));
		if (location == -1)
			std::cout << "Warning: uniform '" << name << "' dosen't exist!\n";
		return mp[name] = location;
	}

	unsigned int shader::createshader(const std::string& vertexShader, const std::string& fragmentShader) {
		unsigned int program = glCreateProgram();
		unsigned int vs = compileshader(GL_VERTEX_SHADER, vertexShader);
		unsigned int fs = compileshader(GL_FRAGMENT_SHADER, fragmentShader);

		glAttachShader(program, vs);
		glAttachShader(program, fs);
		glLinkProgram(program);
		glValidateProgram(program);

		return program;
	}

	template<>
	void shader::set_uniform_array<int>(const std::string & name, const int * data, int count) {
		GLCall(glUniform1iv(getuniformlocation(name), count, data));
	}

	template<>
	void shader::set_uniform_array<float>(const std::string & name, const float * data, int count) {
		GLCall(glUniform1fv(getuniformlocation(name), count, data));
	}

	shadersource shader::parseshader(const std::string &filePath) {
		std::ifstream stream(filePath);
		enum class ShaderType {
			NONE = -1, VERTEX = 0, FRAGMENT = 1
		};
		std::stringstream ss[2];
		ShaderType type = ShaderType::NONE;
		std::string line;

		while (getline(stream, line)) {
			if (line.find("shader") != std::string::npos) {
				if (line.find("vertex") != std::string::npos)
					type = ShaderType::VERTEX;
				else
					type = ShaderType::FRAGMENT;
			}
			else {
				ss[(int)type] << line << '\n';
			}
		}
		return{ ss[0].str() ,ss[1].str() };
	}

	unsigned int shader::compileshader(unsigned int type, const std::string & source) {
		unsigned int id = glCreateShader(type);
		const char* src = source.c_str();
		glShaderSource(id, 1, &src, nullptr);
		glCompileShader(id);

		int res;
		glGetShaderiv(id, GL_COMPILE_STATUS, &res);
		if (!res) {
			int len;
			glGetShaderiv(id, GL_INFO_LOG_LENGTH, &len);
			char* message = (char*)alloca(len);
			glGetShaderInfoLog(id, len, &len, message);
			std::cout << "Failed to compile " << (type == GL_VERTEX_SHADER ? "vertex" : "fragment") << " shader!\n";
			std::cout << message << '\n';
			glDeleteShader(id);
			return 0;
		}
		return id;
	}
}