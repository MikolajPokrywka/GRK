#version 430 core

uniform vec3 objectColor;
//uniform vec3 lightDir;
uniform vec3 lightPos;
uniform vec3 cameraPos;

in vec3 interpNormal;
in vec3 fragPos;
in vec3 vertexPos;
void main()
{
	vec3 lightDir = normalize(lightPos-fragPos);
	vec3 V = normalize(cameraPos-fragPos);
	vec3 normal = normalize(interpNormal);
	vec3 R = reflect(-lightDir,normal);
	
	float specular = pow(max(0,dot(R,V)),10);
	float diffuse = max(0,dot(normal,lightDir));
	float ambient = 0.1;
	float sin_y = sin(vertexPos.y);
	vec3 objectColor2 = vec3(0, 0, 1);
	if (sin(3.14 * vertexPos.x / 0.3) > 0) {
		gl_FragColor.rgb = mix(objectColor, objectColor * diffuse + vec3(1) * specular, 1.0 - ambient);
	}
	else {
		gl_FragColor.rgb = mix(objectColor2, objectColor2 * diffuse + vec3(1) * specular, 1.0 - ambient);
	}
	gl_FragColor.a = 1.0;
}
