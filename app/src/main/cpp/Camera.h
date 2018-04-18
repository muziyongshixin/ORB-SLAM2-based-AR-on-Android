//
// Created by ys on 2017/6/20.
//

#ifndef SINGLEMARKER_CAMERA_H
#define SINGLEMARKER_CAMERA_H

#include <opencv2/core/core.hpp>

using namespace cv;


class Camera {
public:
    Camera(){
        SetDefaultPara();
    };
    Mat camMatrix,distCoeffs;
    void SetDefaultPara();

    Point2f Pixel2Cam ( const Point2f& p, const Mat& K );
    Point2f Cam2Pixel(const Mat& p, const Mat& K);

};


#endif //SINGLEMARKER_CAMERA_H
