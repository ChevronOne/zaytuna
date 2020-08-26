#version 130

uniform samplerCube texSampler;
in vec3 TexCoord;
out vec4 color;

void main()
{ 
    color = texture(texSampler, TexCoord);
}
