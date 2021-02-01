#version 430 core

layout(location = 0) in vec3 vertexPosition;
layout(location = 1) in vec2 vertexTexCoord;
layout(location = 2) in vec3 vertexNormal;
layout(location = 3) in vec3 vertexTangent;
layout(location = 4) in vec3 vertexBitangent;

uniform mat4 modelViewProjectionMatrix;
uniform mat4 modelMatrix;

uniform vec3 lightDir;
uniform vec3 cameraPos;
uniform float time;


out vec2 interpTexCoord;

out vec3 lightDirTs;
out vec3 viewDirTS;

out VS_OUT
{
    vec3 v_Normal;
    vec2 v_TexCoord;
    vec3 v_FragPos;
} vs_out;


void main()
{
	vec3 vertPos;
	vertPos = (modelMatrix * vec4(vertexPosition, 1.0)).xyz;

	vec3 normal = vec3(modelMatrix * vec4(vertexNormal, 0.0));
	vec3 tangent = vec3(modelMatrix * vec4(vertexTangent, 0.0));
	vec3 bitangent = vec3(modelMatrix * vec4(vertexBitangent, 0.0));

	mat3 TBN = transpose(mat3(tangent, bitangent, normal));
	vec3 viewDir = normalize(cameraPos - vertPos);
	lightDirTs = TBN * lightDir;
	viewDirTS = TBN * viewDir;
	//interpNormal = (modelMatrix * vec4(vertexNormal, 0.0)).xyz;
	gl_Position = modelViewProjectionMatrix * vec4(vertexPosition, 1.0);

	interpTexCoord = vertexTexCoord;
	//vs_out.v_Normal = normalize(mat3(transpose(inverse(modelMatrix))) * vertexNormal);
	//vs_out.v_TexCoord = vertexTexCoord;
	//vs_out.v_FragPos = vec3(modelMatrix * vec4(vertexPosition, 1.0));
}
