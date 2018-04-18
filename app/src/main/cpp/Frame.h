//
// Created by ys on 2017/6/20.
//

#ifndef SINGLEMARKER_FRAME_H
#define SINGLEMARKER_FRAME_H


#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "Marker.h"

using namespace cv;
using namespace std;

class Frame{
public:

    Frame();

    Frame(Mat _img){
        img = _img.clone();
        cvtColor(img, gray, CV_BGR2GRAY);
        threshold(gray, thresh, 125, 255, THRESH_BINARY_INV);
    }
    Camera camera;
    Mat img,gray,thresh;
    int detect_number,track_number;
    float perimeter(const vector<Point2f> &a);
    void DetectMarkers(vector<Marker>& markers,bool track_flag);
    void DetectSingleMarker(Marker& marker,bool track_flag);



};


#endif //SINGLEMARKER_FRAME_H
