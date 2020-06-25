#version 130

in vec3 vertPosition;
in vec3 color;
in vec3 normal;

uniform mat4 transformMat;

out vec3 vertCol;

void main()
{
	vertCol = color;
	gl_Position = transformMat * vec4(vertPosition, 1.0f);
}
