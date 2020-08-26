#version 130

uniform sampler2D texSampler;
in vec2 TexCoord;
in vec4 light;
out vec4 color;

void main()
{ 
    color = light * texture(texSampler, TexCoord);
}
