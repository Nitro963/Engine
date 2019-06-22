#shader vertex

#version 330 core
layout (location = 0) in vec3 position;
out vec3 pos;

uniform mat4 view;
uniform mat4 model;
uniform mat4 projection;

void main(){
	gl_Position = projection * view * model * vec4(position, 1.0f);
	pos = position;
}

#shader fragment

#version 330 core
in vec3 pos;
out vec4 FragColor;

void main(){
	if(pos.y < 0)
		FragColor = vec4(1.f);
	else
		if(pos.x < 0)
			FragColor = vec4(1.f, 0.f, 0.f, 1.f);
		else
			FragColor = vec4(0, 1, 0, 1);
};