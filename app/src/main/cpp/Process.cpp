//
// Created by ys on 2017/6/20.
//

#include "Process.h"

bool Process::Run() {
    Frame frame(*pImg);
    Marker marker;
    static bool track_flag = false;

    frame.DetectSingleMarker(marker,track_flag);
    if(marker.id != -1){
        DrawSingleMarkers(marker);
        marker2origincamera = marker.world2camera;
        for(int i = 0; i < 4 ;i++)
            for(int j = 0; j < 4; j++){
                matrix[i*4+j] = marker.world2camera.at<float>(i,j);
            }
        return true;
    }
    else{
        return false;
    }

}
void Process::DrawMarkers(vector<Marker> markers){
    for(size_t i =0 ;i < markers.size();i++){
        for(size_t j = 0;j < markers[i].corners.size();j++){
            Point2f p = markers[i].corners[j];
            circle(*pImg, p, 3, Scalar(0, 255, 0), -1);
            line(*pImg, markers[i].axis_points[0], markers[i].axis_points[j], Scalar(255, 0, 0), 10);

        }
        char text[512];
        sprintf(text, "%d", markers[i].id);
        putText(*pImg, text, markers[i].axis_points[0], FONT_HERSHEY_SIMPLEX, 2, Scalar(0, 255, 0), 3);
    }
}

void Process::DrawSingleMarkers(Marker marker){

    for(size_t j = 0;j < marker.corners.size();j++){
        Point2f p = marker.corners[j];
        circle(*pImg, p, 3, Scalar(0, 255, 0), -1);
        line(*pImg, marker.axis_points[0], marker.axis_points[j], Scalar(255, 0, 0), 10);

    }
    char text[512];
    sprintf(text, "%d", marker.id);
    putText(*pImg, text, marker.axis_points[0], FONT_HERSHEY_SIMPLEX, 2, Scalar(0, 255, 0), 3);

}