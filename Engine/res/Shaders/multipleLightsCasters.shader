#shader vertex

#version 330 core

layout(location = 0) in vec3 position;
layout(location = 1) in vec3 normal;

out VS_OUT{
	vec3 vPos;
	vec3 vNormal;
} vs_out;

uniform mat4 view;
uniform mat4 model;
uniform mat4 projection;

void main() {
	gl_Position = projection * view * model * vec4(position, 1.f);
	vs_out.vPos = vec3(model * vec4(position, 1.0));
	vs_out.vNormal = mat3(transpose(inverse(model))) * normal;
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
} fs_in;

out vec4 fragmentColor;

uniform vec3 view_pos;
uniform DirLight dirLight[5];
uniform SpotLight spotLight[5];
uniform Material material;
uniform int NdirLight;
uniform int NspotLight;
uniform vec4 uColor;

vec3 CalcDirLight(DirLight light, vec3 normal, vec3 view_dir);
vec3 CalcSpotLight(SpotLight light, vec3 normal, vec3 fragPos, vec3 view_dir);

void main(){
	vec3 normal = normalize(fs_in.vNormal);
	vec3 view_dir = normalize(view_pos - fs_in.vPos);

	vec3 result = vec3(0.f, 0.f, 0.f);
	for(int i = 0; i < NdirLight; i++)
		result += CalcDirLight(dirLight[i], normal, view_dir);
	for (int i = 0; i < NspotLight; i++)
		result += CalcSpotLight(spotLight[i], normal, fs_in.vPos, view_dir);

	fragmentColor = result;
}

vec3 CalcDirLight(DirLight light, vec3 normal, vec3 view_dir){
	vec3 lightDir = normalize(-light.direction);
	
	float diff = max(dot(normal, lightDir), 0.0);
	
	vec3 halfway_dir = normalize(lightDir + view_dir);
	float spec = pow(max(dot(normal, halfway_dir), 0.0), material.shininess);
	vec3 ambient = light.ambient * uColor;
	vec3 diffuse = light.diffuse * diff * uColor;
	vec3 specular = light.specular * spec * uColor;
	return (ambient + diffuse + specular);
}

vec3 CalcSpotLight(SpotLight light, vec3 normal, vec3 fragPos, vec3 view_dir){
	vec3 lightDir = normalize(light.position - fragPos);
	// diffuse shading
	float diff = max(dot(normal, lightDir), 0.0);
	// specular shading
	vec3 halfway_dir = normalize(lightDir + view_dir);
	float spec = pow(max(dot(normal, halfway_dir), 0.0), material.shininess);
	// attenuation
	float distance = length(light.position - fragPos);
	float attenuation = 1.0 / (light.constant + light.linear * distance + light.quadratic * (distance * distance));
	// spotlight intensity
	float theta = dot(lightDir, normalize(-light.direction));
	float epsilon = light.cutOff - light.outerCutOff;
	float intensity = clamp((theta - light.outerCutOff) / epsilon, 0.0, 1.0);
	// combine results
	vec3 ambient = light.ambient * uColor;
	vec3 diffuse = light.diffuse * diff * uColor;
	vec3 specular = light.specular * spec * uColor;

	ambient *= attenuation * intensity;
	diffuse *= attenuation * intensity;
	specular *= attenuation * intensity;
	return (ambient + diffuse + specular);
}