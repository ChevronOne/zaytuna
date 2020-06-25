#version 130

uniform sampler2D texSampler;
in vec2 TexCoord;
out vec4 color;

void main()
{ 
    color = texture(texSampler, TexCoord);
}
