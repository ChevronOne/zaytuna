#version 130

in vec3 vertPos;

uniform mat4 transformMat;

out vec3 TexCoord;

void main()
{
    gl_Position = (transformMat * vec4(vertPos, 1.0f)).xyww;
	TexCoord = vertPos;
}
