//
// Created by ys on 2017/6/20.
//

#include "Camera.h"


void Camera::SetDefaultPara(){


    camMatrix = (Mat_<float>(3, 3) <<
                                   253.8520   ,   0  , 158.8690 ,
        0  ,243.5045   ,   118.3277,//153.3180,
        0,      0  ,         1.0000
    );
//    Camera.fx: 247.9092
//    Camera.fy: 238.7612
//    Camera.cx: 156.9728
//    Camera.cy: 118.3277
    distCoeffs = (cv::Mat_<float>(1, 4) << 0.1879, -1.3666,0.0030 ,0.0026);
}

Point2f Camera::Pixel2Cam ( const Point2f& p, const Mat& K )
{
    return Point2f
            (
                    ( p.x - K.at<float>(0,2) ) / K.at<float>(0,0),
                    ( p.y - K.at<float>(1,2) ) / K.at<float>(1,1)
            );
}

Point2f Camera::Cam2Pixel(const Mat& p, const Mat& K)
{
    return Point2f
            (
                    (float(p.at<double>(0, 0)) * K.at<float>(0, 0)+ K.at<float>(0, 2))  ,
                    (float(p.at<double>(1, 0)) * K.at<float>(1, 1) + K.at<float>(1, 2))
            );
}