#version 130

in vec3 vertPos;
in vec3 vertNorm;
in vec2 texCoor;

uniform mat4 transformMat;
uniform mat4 it_transformMat;

out vec2 TexCoord;
out vec4 light;


void main()
{
    gl_Position = transformMat * vec4(vertPos, 1.0);
    vec3 Norm = vec3(it_transformMat *vec4(vertNorm, 1.0));
    light = vec4(vec3(0.6f, 0.6f, 0.6f) // ambient
            +(max(dot(normalize(vec3(5000000.0f, 4000000.0f, -5000000.0f) // sun's position
            -vec3(gl_Position)), Norm), 0.f)*vec3(0.6f, .6f, .6f)), 1.f);
	TexCoord = texCoor;
}
