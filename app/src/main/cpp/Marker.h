//
// Created by ys on 2017/6/20.
//

#ifndef SINGLEMARKER_MARKER_H
#define SINGLEMARKER_MARKER_H

#include <opencv2/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <vector>


#include "Camera.h"


using namespace cv;
using namespace std;

class Marker {
public:
    Marker(){
        id = -1;
    }
    int id;
    Mat world2camera,camera2world;
    vector<Point2f> corners;
    vector<Point2f> axis_points;
    int hammDistMarker18(Mat bits);
    Mat rotate(Mat in);
    int mat2id18(const Mat &bits);
    void FindID(Mat thresh);
    void GetTransformMatrix(Camera camera);
};


#endif //SINGLEMARKER_MARKER_H
