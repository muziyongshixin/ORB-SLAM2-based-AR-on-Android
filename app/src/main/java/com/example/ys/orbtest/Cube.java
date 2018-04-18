package com.example.ys.orbtest;

import android.content.Context;
import android.opengl.GLES20;

import com.example.ys.orbtest.MatrixState;
import com.example.ys.orbtest.util.ShaderHelper;
import com.example.ys.orbtest.util.TextResourceReader;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.FloatBuffer;

public class Cube {

    //顶点坐标
    private FloatBuffer vertexBuffer;
    //颜色坐标
    private FloatBuffer colorBuffer;
    private Context context;
    //float类型的字节数
    private static final int BYTES_PER_FLOAT = 4;
    //共有72个顶点坐标，每个面包含12个顶点坐标
    private static final int POSITION_COMPONENT_COUNT = 12 * 6;
    // 数组中每个顶点的坐标数
    private static final int COORDS_PER_VERTEX = 3;
    // 颜色数组中每个颜色的值数
    private static final int COORDS_PER_COLOR = 4;

    private static final String A_POSITION = "a_Position";
    private static final String A_COLOR = "a_Color";
    private static final String U_MATRIX = "u_Matrix";
    private int uMatrixLocation;
    private int aColorLocation;
    private int aPositionLocation;
    private int program;

    static float vertices[] = {
            //前面
            0, 0, 1.0f,
            1.0f, 1.0f, 1.0f,
            -1.0f, 1.0f, 1.0f,
            0, 0, 1.0f,
            -1.0f, 1.0f, 1.0f,
            -1.0f, -1.0f, 1.0f,
            0, 0, 1.0f,
            -1.0f, -1.0f, 1.0f,
            1.0f, -1.0f, 1.0f,
            0, 0, 1.0f,
            1.0f, -1.0f, 1.0f,
            1.0f, 1.0f, 1.0f,
            //后面
            0, 0, -1.0f,
            1.0f, 1.0f, -1.0f,
            1.0f, -1.0f, -1.0f,
            0, 0, -1.0f,
            1.0f, -1.0f, -1.0f,
            -1.0f, -1.0f, -1.0f,
            0, 0, -1.0f,
            -1.0f, -1.0f, -1.0f,
            -1.0f, 1.0f, -1.0f,
            0, 0, -1.0f,
            -1.0f, 1.0f, -1.0f,
            1.0f, 1.0f, -1.0f,
            //左面
            -1.0f, 0, 0,
            -1.0f, 1.0f, 1.0f,
            -1.0f, 1.0f, -1.0f,
            -1.0f, 0, 0,
            -1.0f, 1.0f, -1.0f,
            -1.0f, -1.0f, -1.0f,
            -1.0f, 0, 0,
            -1.0f, -1.0f, -1.0f,
            -1.0f, -1.0f, 1.0f,
            -1.0f, 0, 0,
            -1.0f, -1.0f, 1.0f,
            -1.0f, 1.0f, 1.0f,
            //右面
            1.0f, 0, 0,
            1.0f, 1.0f, 1.0f,
            1.0f, -1.0f, 1.0f,
            1.0f, 0, 0,
            1.0f, -1.0f, 1.0f,
            1.0f, -1.0f, -1.0f,
            1.0f, 0, 0,
            1.0f, -1.0f, -1.0f,
            1.0f, 1.0f, -1.0f,
            1.0f, 0, 0,
            1.0f, 1.0f, -1.0f,
            1.0f, 1.0f, 1.0f,
            //上面
            0, 1.0f, 0,
            1.0f, 1.0f, 1.0f,
            1.0f, 1.0f, -1.0f,
            0, 1.0f, 0,
            1.0f, 1.0f, -1.0f,
            -1.0f, 1.0f, -1.0f,
            0, 1.0f, 0,
            -1.0f, 1.0f, -1.0f,
            -1.0f, 1.0f, 1.0f,
            0, 1.0f, 0,
            -1.0f, 1.0f, 1.0f,
            1.0f, 1.0f, 1.0f,
            //下面
            0, -1.0f, 0,
            1.0f, -1.0f, 1.0f,
            -1.0f, -1.0f, 1.0f,
            0, -1.0f, 0,
            -1.0f, -1.0f, 1.0f,
            -1.0f, -1.0f, -1.0f,
            0, -1.0f, 0,
            -1.0f, -1.0f, -1.0f,
            1.0f, -1.0f, -1.0f,
            0, -1.0f, 0,
            1.0f, -1.0f, -1.0f,
            1.0f, -1.0f, 1.0f
    };

    static float smallvertix[] = {0.0f,0.0f,-0.1f,0.1f,0.1f,-0.1f,-0.1f,0.1f,-0.1f,0.0f,0.0f,-0.1f,-0.1f,0.1f,-0.1f,-0.1f,-0.1f,-0.1f,0.0f,0.0f,-0.1f,-0.1f,-0.1f,-0.1f,0.1f,-0.1f,-0.1f,0.0f,0.0f,-0.1f,0.1f,-0.1f,-0.1f,0.1f,0.1f,-0.1f,0.0f,0.0f,0.1f,0.1f,0.1f,0.1f,0.1f,-0.1f,0.1f,0.0f,0.0f,0.1f,0.1f,-0.1f,0.1f,-0.1f,-0.1f,0.1f,0.0f,0.0f,0.1f,-0.1f,-0.1f,0.1f,-0.1f,0.1f,0.1f,0.0f,0.0f,0.1f,-0.1f,0.1f,0.1f,0.1f,0.1f,0.1f,-0.1f,0.0f,-0.0f,-0.1f,0.1f,-0.1f,-0.1f,0.1f,0.1f,-0.1f,0.0f,-0.0f,-0.1f,0.1f,0.1f,-0.1f,-0.1f,0.1f,-0.1f,0.0f,-0.0f,-0.1f,-0.1f,0.1f,-0.1f,-0.1f,-0.1f,-0.1f,0.0f,-0.0f,-0.1f,-0.1f,-0.1f,-0.1f,0.1f,-0.1f,0.1f,0.0f,-0.0f,0.1f,0.1f,-0.1f,0.1f,-0.1f,-0.1f,0.1f,0.0f,-0.0f,0.1f,-0.1f,-0.1f,0.1f,-0.1f,0.1f,0.1f,0.0f,-0.0f,0.1f,-0.1f,0.1f,0.1f,0.1f,0.1f,0.1f,0.0f,-0.0f,0.1f,0.1f,0.1f,0.1f,0.1f,-0.1f,0.0f,0.1f,-0.0f,0.1f,0.1f,-0.1f,0.1f,0.1f,0.1f,0.0f,0.1f,-0.0f,0.1f,0.1f,0.1f,-0.1f,0.1f,0.1f,0.0f,0.1f,-0.0f,-0.1f,0.1f,0.1f,-0.1f,0.1f,-0.1f,0.0f,0.1f,-0.0f,-0.1f,0.1f,-0.1f,0.1f,0.1f,-0.1f,0.0f,-0.1f,-0.0f,0.1f,-0.1f,-0.1f,-0.1f,-0.1f,-0.1f,0.0f,-0.1f,-0.0f,-0.1f,-0.1f,-0.1f,-0.1f,-0.1f,0.1f,0.0f,-0.1f,-0.0f,-0.1f,-0.1f,0.1f,0.1f,-0.1f,0.1f,0.0f,-0.1f,-0.0f,0.1f,-0.1f,0.1f,0.1f,-0.1f,-0.1f};

    //顶点颜色值数组，每个顶点4个色彩值RGBA
    static float colors[] = new float[]{
            //前面
            1, 1, 1, 1,//中间为白色
            1, 0, 0, 1,
            1, 0, 0, 1,
            1, 1, 1, 1,//中间为白色
            1, 0, 0, 1,
            1, 0, 0, 1,
            1, 1, 1, 1,//中间为白色
            1, 0, 0, 1,
            1, 0, 0, 1,
            1, 1, 1, 1,//中间为白色
            1, 0, 0, 1,
            1, 0, 0, 1,
            //后面
            1, 1, 1, 1,//中间为白色
            0, 0, 1, 1,
            0, 0, 1, 1,
            1, 1, 1, 1,//中间为白色
            0, 0, 1, 1,
            0, 0, 1, 1,
            1, 1, 1, 1,//中间为白色
            0, 0, 1, 1,
            0, 0, 1, 1,
            1, 1, 1, 1,//中间为白色
            0, 0, 1, 1,
            0, 0, 1, 1,
            //左面
            1, 1, 1, 1,//中间为白色
            1, 0, 1, 1,
            1, 0, 1, 1,
            1, 1, 1, 1,//中间为白色
            1, 0, 1, 1,
            1, 0, 1, 1,
            1, 1, 1, 1,//中间为白色
            1, 0, 1, 1,
            1, 0, 1, 1,
            1, 1, 1, 1,//中间为白色
            1, 0, 1, 1,
            1, 0, 1, 1,
            //右面
            1, 1, 1, 1,//中间为白色
            1, 1, 0, 1,
            1, 1, 0, 1,
            1, 1, 1, 1,//中间为白色
            1, 1, 0, 1,
            1, 1, 0, 1,
            1, 1, 1, 1,//中间为白色
            1, 1, 0, 1,
            1, 1, 0, 1,
            1, 1, 1, 1,//中间为白色
            1, 1, 0, 1,
            1, 1, 0, 1,
            //上面
            1, 1, 1, 1,//中间为白色
            0, 1, 0, 1,
            0, 1, 0, 1,
            1, 1, 1, 1,//中间为白色
            0, 1, 0, 1,
            0, 1, 0, 1,
            1, 1, 1, 1,//中间为白色
            0, 1, 0, 1,
            0, 1, 0, 1,
            1, 1, 1, 1,//中间为白色
            0, 1, 0, 1,
            0, 1, 0, 1,
            //下面
            1, 1, 1, 1,//中间为白色
            0, 1, 1, 1,
            0, 1, 1, 1,
            1, 1, 1, 1,//中间为白色
            0, 1, 1, 1,
            0, 1, 1, 1,
            1, 1, 1, 1,//中间为白色
            0, 1, 1, 1,
            0, 1, 1, 1,
            1, 1, 1, 1,//中间为白色
            0, 1, 1, 1,
            0, 1, 1, 1,
    };

    public Cube(Context context) {
        this.context = context;

        vertexBuffer = ByteBuffer
                .allocateDirect(vertices.length * BYTES_PER_FLOAT)
                .order(ByteOrder.nativeOrder())
                .asFloatBuffer();
        // 把坐标们加入FloatBuffer中
        vertexBuffer.put(vertices);
        // 设置buffer，从第一个坐标开始读
        vertexBuffer.position(0);

        //颜色buffer
        colorBuffer = ByteBuffer
                .allocateDirect(colors.length * BYTES_PER_FLOAT)
                .order(ByteOrder.nativeOrder())
                .asFloatBuffer();
        colorBuffer.put(colors);
        colorBuffer.position(0);

        getProgram();

        aColorLocation = GLES20.glGetAttribLocation(program, A_COLOR);
        aPositionLocation = GLES20.glGetAttribLocation(program, A_POSITION);
        uMatrixLocation = GLES20.glGetUniformLocation(program, U_MATRIX);

        //---------传入顶点数据数据
        GLES20.glVertexAttribPointer(aPositionLocation, COORDS_PER_VERTEX,
                GLES20.GL_FLOAT, false, 0, vertexBuffer);
        GLES20.glEnableVertexAttribArray(aPositionLocation);
        //---------传入颜色数据
        GLES20.glVertexAttribPointer(aColorLocation, COORDS_PER_COLOR,
                GLES20.GL_FLOAT, false, 0, colorBuffer);
        GLES20.glEnableVertexAttribArray(aColorLocation);
    }

    //获取program
    private void getProgram() {
        //获取顶点着色器文本
        String vertexShaderSource = TextResourceReader
                .readTextFileFromResource(context, R.raw.simple_vertex_shader);
        //获取片段着色器文本
        String fragmentShaderSource = TextResourceReader
                .readTextFileFromResource(context, R.raw.simple_fragment_shader);
        //获取program的id
        program = ShaderHelper.buildProgram(vertexShaderSource, fragmentShaderSource);
        GLES20.glUseProgram(program);
    }

    public void draw() {
        GLES20.glUniformMatrix4fv(uMatrixLocation, 1, false, MatrixState.final_matrix, 0);
        GLES20.glDrawArrays(GLES20.GL_TRIANGLES, 0, POSITION_COMPONENT_COUNT);
        System.out.println("one look at matrix is bellow======================");
        System.out.println(MatrixState.model_view_matrix[0] + " " + MatrixState.model_view_matrix[1] + " " + MatrixState.model_view_matrix[2] + " " + MatrixState.model_view_matrix[3]);
        System.out.println(MatrixState.model_view_matrix[4] + " " + MatrixState.model_view_matrix[5] + " " + MatrixState.model_view_matrix[6] + " " + MatrixState.model_view_matrix[7]);
        System.out.println(MatrixState.model_view_matrix[8] + " " + MatrixState.model_view_matrix[9] + " " + MatrixState.model_view_matrix[10] + " " + MatrixState.model_view_matrix[11]);
        System.out.println(MatrixState.model_view_matrix[12] + " " + MatrixState.model_view_matrix[13] + " " + MatrixState.model_view_matrix[14] + " " + MatrixState.model_view_matrix[15]);


        System.out.println("one projection matrix is bellow======================");
        System.out.println(MatrixState.projection_matrix[0] + " " + MatrixState.projection_matrix[1] + " " + MatrixState.projection_matrix[2] + " " + MatrixState.projection_matrix[3]);
        System.out.println(MatrixState.projection_matrix[4] + " " + MatrixState.projection_matrix[5] + " " + MatrixState.projection_matrix[6] + " " + MatrixState.projection_matrix[7]);
        System.out.println(MatrixState.projection_matrix[8] + " " + MatrixState.projection_matrix[9] + " " + MatrixState.projection_matrix[10] + " " + MatrixState.projection_matrix[11]);
        System.out.println(MatrixState.projection_matrix[12] + " " + MatrixState.projection_matrix[13] + " " + MatrixState.projection_matrix[14] + " " + MatrixState.projection_matrix[15]);

        System.out.println("one final matrix is bellow======================");
        System.out.println(MatrixState.final_matrix[0] + " " + MatrixState.final_matrix[1] + " " + MatrixState.final_matrix[2] + " " + MatrixState.final_matrix[3]);
        System.out.println(MatrixState.final_matrix[4] + " " + MatrixState.final_matrix[5] + " " + MatrixState.final_matrix[6] + " " + MatrixState.final_matrix[7]);
        System.out.println(MatrixState.final_matrix[8] + " " + MatrixState.final_matrix[9] + " " + MatrixState.final_matrix[10] + " " + MatrixState.final_matrix[11]);
        System.out.println(MatrixState.final_matrix[12] + " " + MatrixState.final_matrix[13] + " " + MatrixState.final_matrix[14] + " " + MatrixState.final_matrix[15]);
    }
}
