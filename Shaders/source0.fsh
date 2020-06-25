#version 130

in vec3 vertCol;

void main()
{ 

	gl_FragColor = vec4(vertCol, 1.0f);

}

