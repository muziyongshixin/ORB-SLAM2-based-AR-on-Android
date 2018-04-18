precision mediump float;
varying vec4 vPosition;//接收从顶点着色器过来的顶点位置
uniform sampler2D u_TextureUnit;      	 								
varying vec2 v_TextureCoordinates;
void main()                         
{
   float uR = 0.6;//球的半径
   vec4 color;
   float n = 4.0;//分为n层n列n行
   float span = 2.0*uR/n;//正方形长度
   //计算行列层数
   int i = int((vPosition.x + uR)/span);//行数
   int j = int((vPosition.y + uR)/span);//层数
   int k = int((vPosition.z + uR)/span);//列数
   int colorType = int(mod(float(i+j+k),2.0));
   if(colorType == 1) {//奇数时为绿色
   		color = vec4(0.2,1.0,0.129,0);
   }
   else {//偶数时为白色
   		color = vec4(1.0,1.0,1.0,0);//白色
   }
   //将计算出的颜色给此片元
   gl_FragColor=color;
   gl_FragColor = texture2D(u_TextureUnit, v_TextureCoordinates);  
}     