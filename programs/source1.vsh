#version 130

in vec3 vertPos;
in vec3 vertNorm;
in vec2 texCoor;

uniform mat4 transformMat;

out vec2 TexCoord;
out vec4 light;


void main()
{
    gl_Position = transformMat * vec4(vertPos, 1.0);
    light = vec4(vec3(0.85f, 0.85f, 0.85f) // ambient
            +(max(dot(normalize(vec3(5000000.0f, 4000000.0f, -5000000.0f) // sun's position
            -vertPos), vertNorm), 0.f)*vec3(0.4f, .4f, .4f)), 1.f);
	TexCoord = texCoor;
}
