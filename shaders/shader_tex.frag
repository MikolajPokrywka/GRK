#version 430 core

uniform sampler2D textureSampler;
uniform sampler2D normalSampler;
uniform vec3 lightDir;
uniform vec3 cameraPos;

in vec2 interpTexCoord;

in vec3 lightDirTs;
in vec3 viewDirTS;

void main()
{
	vec3 vertPos;
	vec3 lightDirTs = normalize(lightDirTs);
	vec3 L = -lightDirTs;
	vec3 V = normalize(viewDirTS);
	vec3 N = normalize(texture2D(normalSampler, interpTexCoord).rgb * 2 - 1);
	vec3 R = reflect(-normalize(L), N);

	float diffuse = max(0, dot(N, L));
	
	float specular_pow = 10000;
	float specular = pow(max(0, dot(R, V)), specular_pow);
	vec3 color = texture2D(textureSampler, interpTexCoord).rgb;
	vec3 lightColor = vec3(0.5);

	vec3 shadedColor = color * diffuse + lightColor * specular;
	
	float ambient = 0.2;

	gl_FragColor = vec4(mix(color, shadedColor, 1.0 - ambient), 1.0);
}
