#version 430 core

uniform sampler2D textureSampler;
uniform sampler2D normalSampler;
uniform vec3 lightDir;
uniform vec3 cameraPos;

in vec2 interpTexCoord;

in vec3 lightDirTs;
in vec3 lightDir2Ts;
in vec3 viewDirTS;

void main()
{
	vec3 vertPos;
	vec3 L = normalize(-lightDirTs);
	vec3 V = normalize(viewDirTS);
	vec3 N = normalize(texture2D(normalSampler, interpTexCoord).rgb * 2 - 1);
	vec3 R = reflect(-normalize(L), N);

	float diffuse = max(0, dot(N, L));

	//drugie zrodlo swiatla
	vec3 L2 = normalize(-lightDir2Ts);
	vec3 V2 = normalize(viewDirTS);
	vec3 R2 = reflect(-normalize(L2), N);

	float diffuse2 = max(0, dot(N, L2));
	
	float specular_pow = 10;
	float specular = pow(max(0, dot(R, V)), specular_pow);
	float specular2 = pow(max(0, dot(R2, V2)), specular_pow);
	vec3 color = texture2D(textureSampler, interpTexCoord).rgb;
	vec3 lightColor = vec3(1);

	//diffuse dodac, lighcolor dwa i * specular i dodac
	vec3 shadedColor = color * (diffuse+diffuse2) + lightColor * specular + lightColor * specular2;
	
	float ambient = 0.2;

	gl_FragColor = vec4(mix(color, shadedColor, 1.0 - ambient), 1.0);
}
