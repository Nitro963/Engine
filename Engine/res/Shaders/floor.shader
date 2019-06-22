#shader vertex

#version 330 core
layout(location = 0) in vec3 position;
layout(location = 1) in vec2 texcoord;
out vec3 pos;
out vec2 texCoord;

uniform mat4 view;
uniform mat4 projection;

void main() {
	gl_Position = projection * view * vec4(position, 1.0f);
	pos = position;
	texCoord = texcoord;
}

#shader fragment

#version 330 core
in vec3 pos;
in vec2 texCoord;

out vec4 FragColor;
uniform sampler2D u_tex;

void main() {
	FragColor = texture(u_tex, texCoord);
};