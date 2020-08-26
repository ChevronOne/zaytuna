#version 130

in vec3 vertPos;
in vec3 vecNorm;
in vec2 textCoor;

uniform mat4 transformMat;

out vec2 TexCoord;

void main()
{
    gl_Position = transformMat * vec4(vertPos, 1.0);
	TexCoord = textCoor;
}
