//
// Created by ys on 2017/7/25.
//

#ifndef ORBTEST_PLANE_H
#define ORBTEST_PLANE_H
#include "System.h"

class Plane {
public:
    Plane(){}
    cv::Mat DetectPlane(const cv::Mat Tcw, const std::vector<ORB_SLAM2::MapPoint*> &vMPs, const int iterations);
    void Recompute();

    //normal
    cv::Mat n;
    //origin
    cv::Mat o;
    //arbitrary orientation along normal
    float rang;
    //transformation from world to the plane
    cv::Mat Tpw;
    //MapPoints that define the plane
    std::vector<ORB_SLAM2::MapPoint*> mvMPs;
    //camera pose when the plane was first observed (to compute normal direction)
    cv::Mat mTcw, XC;
};


#endif //ORBTEST_PLANE_H
