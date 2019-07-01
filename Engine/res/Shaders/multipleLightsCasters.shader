#shader vertex

#version 330 core

layout(location = 0) in vec3 position;
layout(location = 1) in vec3 normal;
layout(location = 2) in vec3 color;
out VS_OUT{
	vec3 vPos;
	vec3 vNormal;
	vec3 vColor;
} vs_out;

uniform mat4 view;
uniform mat4 model;
uniform mat4 projection;

void main() {
	gl_Position = projection * view * model * vec4(position, 1.f);
	vs_out.vPos = vec3(model * vec4(position, 1.0));
	vs_out.vNormal = mat3(transpose(inverse(model))) * normal;
	vs_out.vColor = color;
};

#shader fragment

#version 330 core

struct Material {
	float shininess;
};

struct DirLight {
	vec3 direction;

	vec3 ambient;
	vec3 diffuse;
	vec3 specular;
};


struct SpotLight {
	vec3 position;
	vec3 direction;
	float cutOff;
	float outerCutOff;
	float constant;
	float linear;
	float quadratic;

	vec3 ambient;
	vec3 diffuse;
	vec3 specular;
};

in VS_OUT{
	vec3 vPos;
	vec3 vNormal;
	vec3 vColor;
} fs_in;

out vec4 fragmentColor;
uniform vec3 view_pos;
uniform DirLight dirLight[5];
uniform SpotLight spotLight[5];
uniform Material material;
uniform int NdirLight;
uniform int useUniformColor;
uniform vec3 uColor;

vec3 CalcDirLight(DirLight light, vec3 normal, vec3 view_dir, vec3 color);

void main(){
	vec3 normal = normalize(fs_in.vNormal);
	vec3 view_dir = normalize(view_pos - fs_in.vPos);

	vec3 result = vec3(0.f, 0.f, 0.f);
	for (int i = 0; i < NdirLight; i++)
		if(useUniformColor == 1)
			result += CalcDirLight(dirLight[0], normal, view_dir, uColor);
		else
			result += CalcDirLight(dirLight[i], normal, view_dir, fs_in.vColor);

	fragmentColor = vec4(result, 1.f);
}

vec3 CalcDirLight(DirLight light, vec3 normal, vec3 view_dir, vec3 color){
	vec3 lightDir = normalize(-light.direction);
	
	float diff = max(dot(normal, lightDir), 0.0);
	
	vec3 halfway_dir = normalize(lightDir + view_dir);
	float spec = pow(max(dot(normal, halfway_dir), 0.0), material.shininess);
	vec3 ambient = light.ambient * color;
	vec3 diffuse = light.diffuse * diff * color;
	vec3 specular = light.specular * spec * color;
	return (ambient + diffuse + specular);
}