package com.example.ys.orbtest;

/**
 * Created by dell on 2017/12/20.
 */

import android.content.Context;
import android.opengl.GLES20;
import android.opengl.GLSurfaceView.Renderer;
import android.util.Log;

import com.example.ys.orbtest.Cube;
import com.example.ys.orbtest.MatrixState;
import com.example.ys.orbtest.obj.Ball;

import javax.microedition.khronos.egl.EGLConfig;
import javax.microedition.khronos.opengles.GL10;

import static android.opengl.GLES20.glClear;
import static android.opengl.GLES20.glClearColor;
import static android.opengl.GLES20.glViewport;

public class MyRender implements Renderer {

    public static boolean flag=false;
    private Context context;

    private static boolean isSelected=true;

    public MyRender(Context context){
        this.context = context;
    }

    //  Circle circle;
    Cube cube;
    Ball ball;//地球

    public void onSurfaceCreated(GL10 gl, EGLConfig config) {
        Log.w("MyRender","onSurfaceCreated");
        //设置屏幕背景色RGBA
//        glClearColor(0.5f,0.5f,0.5f, 1.0f);
        glClearColor(0.01f,0.01f,0.01f,0.01f);
//      //打开深度检测
        GLES20.glEnable(GLES20.GL_DEPTH_TEST);
//        //打开背面剪裁
        GLES20.glEnable(GLES20.GL_CULL_FACE);
//      circle = new Circle(context);
        cube = new Cube(context);
        ball = new Ball(context);
    }

    public void onSurfaceChanged(GL10 gl, int width, int height) {
        glViewport(0,0,width,height);
        float ratio = (float) width / height;
        //设置投影矩阵
//      circle.projectionMatrix(width, height);
        // 调用此方法计算产生透视投影矩阵
//        MatrixState.setProjectFrustum(-ratio,ratio, -1, 1, 20, 200);
//        // 调用此方法产生摄像机9参数位置矩阵
//        MatrixState.setCamera(-16f, 8f, 45, 0f, 0f, 0f, 0f, 1.0f, 0.0f);
    }

    public void onDrawFrame(GL10 gl) {
//      glClear(GL_COLOR_BUFFER_BIT);
        //清除深度缓冲与颜色缓冲
        glClear( GLES20.GL_DEPTH_BUFFER_BIT | GLES20.GL_COLOR_BUFFER_BIT);

        if(flag){
//            cube.draw();
            ball.draw();
        }

    }



    public void handleTouchPress(float normalizedX, float normalizedY) {
        System.out.println("touchPress++++++++++++++++++++++++++");
        if(Math.abs(normalizedX-MatrixState.x)<=1&&Math.abs(normalizedY-MatrixState.y)<=1)
            isSelected=true;
    }
    public void handleTouchDrag(float normalizedX, float normalizedY) {
        System.out.println(normalizedX + "===========" + normalizedY + "================");
        System.out.println(MatrixState.x+"========"+MatrixState.y);
        if (isSelected)
        {
            MatrixState.x=normalizedX;
            MatrixState.y=normalizedY;
        }
    }
    public void handleTouchUp(float normalizedX,float normalizedY){
        System.out.println("touchup++++++++++++++++++++++++++");
        isSelected=false;
    }
}