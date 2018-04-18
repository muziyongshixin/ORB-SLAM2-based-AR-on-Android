//vertex_shader_ball.glsl
uniform mat4 u_Matrix;//最终的变换矩阵
attribute vec4 a_Position;//顶点位置
varying vec4 vPosition;//用于传递给片元着色器的顶点位置
attribute vec2 a_TextureCoordinates;
varying vec2 v_TextureCoordinates;
void main()                    
{                              
    gl_Position = u_Matrix * a_Position;
    vPosition = a_Position;
    v_TextureCoordinates = a_TextureCoordinates;	 
} 