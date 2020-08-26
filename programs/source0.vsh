#version 130

in vec3 vertPos;
in vec3 vertColor;
//in vec3 vertNorm;

uniform mat4 transformMat;

out vec3 vertCol;

void main()
{
	vertCol = vertColor;
	gl_Position = transformMat * vec4(vertPos, 1.0f);
}
