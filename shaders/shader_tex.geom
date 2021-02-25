#version 430 core
layout (triangles) in;
layout (triangle_strip, max_vertices = 3) out;

in vs_out {
    vec2 texCoords;
} gs_in[];

out vec2 TexCoords; 

uniform float explosionProgress;




vec3 GetNormal() {
   vec3 a = vec3(gl_in[0].gl_Position) - vec3(gl_in[1].gl_Position);
   vec3 b = vec3(gl_in[2].gl_Position) - vec3(gl_in[1].gl_Position);
   return normalize(cross(a, b));
}

vec4 explode(vec4 position, vec3 normal) {
    // odleglosc
    float magnitude = 10.0;
    vec3 direction = normal * ((explosionProgress) / 2.0) * magnitude; 
    return position + vec4(direction, 0.0);
} 

void main() {
    vec3 normal = GetNormal();


    gl_Position = explode(gl_in[0].gl_Position, normal);
    TexCoords = gs_in[0].texCoords;
    EmitVertex();
    gl_Position = explode(gl_in[1].gl_Position, normal);
    TexCoords = gs_in[1].texCoords;
    EmitVertex();
    gl_Position = explode(gl_in[2].gl_Position, normal);
    TexCoords = gs_in[2].texCoords;
    EmitVertex();
    EndPrimitive();
}

