package com.example.ys.orbtest.util;

/**
 * Created by dell on 2017/12/28.
 */

import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.opengl.GLES20;
import android.opengl.GLUtils;
import android.util.Log;

public class TextureHelper {

    public static final String TAG = "TextureHelper";

    public static int loadTexture(Context context, int resourceId, boolean isRepeat){
        /*
         * 第一步 : 创建纹理对象
         */
        final int[] textureObjectId = new int[1];//用于存储返回的纹理对象ID
        GLES20.glGenTextures(1,textureObjectId, 0);
        if(textureObjectId[0] == 0){//若返回为0,,则创建失败
            if(LoggerConfig.ON){
                Log.w(TAG,"Could not generate a new Opengl texture object");
            }
            return 0;
        }
        /*
         * 第二步: 加载位图数据并与纹理绑定
         */
        final BitmapFactory.Options options = new BitmapFactory.Options();
        options.inScaled = false;//Opengl需要非压缩形式的原始数据
        final Bitmap bitmap = BitmapFactory.decodeResource(context.getResources(),resourceId, options);
        if(bitmap == null){
            if(LoggerConfig.ON){
                Log.w(TAG,"ResourceId:"+resourceId+"could not be decoded");
            }
            GLES20.glDeleteTextures(1, textureObjectId, 0);
            return 0;
        }
        GLES20.glBindTexture(GLES20.GL_TEXTURE_2D,textureObjectId[0]);//通过纹理ID进行绑定

        if(isRepeat){
            GLES20.glTexParameterf(GLES20.GL_TEXTURE_2D, GLES20.GL_TEXTURE_WRAP_S, GLES20.GL_REPEAT);
            GLES20.glTexParameterf(GLES20.GL_TEXTURE_2D, GLES20.GL_TEXTURE_WRAP_T, GLES20.GL_REPEAT);
        }

        /*
         * 第三步: 设置纹理过滤
         */
        //设置缩小时为三线性过滤
        GLES20.glTexParameteri(GLES20.GL_TEXTURE_2D, GLES20.GL_TEXTURE_MIN_FILTER, GLES20.GL_LINEAR_MIPMAP_LINEAR);
        //设置放大时为双线性过滤
        GLES20.glTexParameteri(GLES20.GL_TEXTURE_2D, GLES20.GL_TEXTURE_MAG_FILTER, GLES20.GL_LINEAR);
        /*
         * 第四步: 加载纹理到Opengl并返回ID
         */
        GLUtils.texImage2D(GLES20.GL_TEXTURE_2D, 0, bitmap, 0);
        bitmap.recycle();
        GLES20.glGenerateMipmap(GLES20.GL_TEXTURE_2D);
        return textureObjectId[0];
    }
}