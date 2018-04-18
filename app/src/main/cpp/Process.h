//
// Created by ys on 2017/6/20.
//

#ifndef SINGLEMARKER_PROCESS_H
#define SINGLEMARKER_PROCESS_H

#include "Frame.h"

class Process {
public:
    Process(Mat* _pImg){
        pImg = _pImg;
    }
    Mat* pImg;
    cv::Mat marker2origincamera;
    float matrix[16];
    bool Run();
    void DrawMarkers(vector<Marker> markers);
    void DrawSingleMarkers(Marker marker);
};


#endif //SINGLEMARKER_PROCESS_H
