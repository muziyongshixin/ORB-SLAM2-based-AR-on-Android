//
// Created by ys on 2017/6/20.
//

#include "Frame.h"
float Frame::perimeter(const vector<Point2f> &a)
{
    float sum = 0, dx, dy;

    for (size_t i = 0; i<a.size(); i++)
    {
        size_t i2 = (i + 1) % a.size();

        dx = a[i].x - a[i2].x;
        dy = a[i].y - a[i2].y;

        sum += sqrt(dx*dx + dy*dy);
    }

    return sum;
}

void Frame::DetectMarkers(vector<Marker>& markers,bool track_flag){
    vector<vector<Point> >allcontours;
    findContours(thresh, allcontours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
    vector<vector<Point> >contours;
    contours.clear();
    for (size_t i = 0; i<allcontours.size(); i++)
    {
        int contourSize = allcontours[i].size();
        if (contourSize > thresh.cols / 10)
            //if (contourSize > gray.cols/5)
        {
            contours.push_back(allcontours[i]);
        }
    }
    vector<Point> approxCurve;
    vector<vector<Point2f> > good_points;

    for (size_t i = 0; i < contours.size(); i++) {
        double eps = contours[i].size() * 0.05;
        approxPolyDP(contours[i], approxCurve, eps, true);

        if (approxCurve.size() < 4)
            continue;

        if (!isContourConvex(approxCurve))
            continue;

        float minDist = std::numeric_limits<float>::max();
        for (int i = 0; i < 4; i++)
        {
            Point side = approxCurve[i] - approxCurve[(i + 1) % 4];
            float squaredSideLength = side.dot(side);
            minDist = min(minDist, squaredSideLength);
        }

        if (minDist < 100)
            continue;

        vector<Point2f> points;

        for (int i = 0; i< 4; i++) {
            points.push_back(Point2f(approxCurve[i].x, approxCurve[i].y));
        }
        Point v1 = points[1] - points[0];
        Point v2 = points[2] - points[0];
        double cross_result = v1.x*v2.y - v2.x*v1.y;
        if (cross_result < 0)
            swap(points[1], points[3]);

        good_points.push_back(points);

    }


    vector<pair<int, int> > tooNearCandidates;
    for (size_t i = 0; i<good_points.size(); i++)
    {
        const vector<Point2f>& points1 = good_points[i];

        for (size_t j = i + 1; j<good_points.size(); j++)
        {
            const vector<Point2f>& points2 = good_points[j];

            float distSquared = 0;

            for (int c = 0; c < 4; c++)
            {
                Point v = points1[c] - points2[c];
                distSquared += v.dot(v);
            }

            distSquared /= 4;

            if (distSquared < 100)
            {
                tooNearCandidates.push_back(pair<int, int>(i, j));
            }
        }
    }

    vector<bool> removalMask(good_points.size(), false);

    for (size_t i = 0; i<tooNearCandidates.size(); i++)
    {
        float p1 = perimeter(good_points[tooNearCandidates[i].first]);
        float p2 = perimeter(good_points[tooNearCandidates[i].second]);

        size_t removalIndex;
        if (p1 > p2)
            removalIndex = tooNearCandidates[i].second;
        else
            removalIndex = tooNearCandidates[i].first;

        removalMask[removalIndex] = true;
    }
    vector<Marker > temp_markers;
    for (size_t i = 0; i<good_points.size(); i++)
    {

        if (!removalMask[i]){
            Marker m;
            for(size_t j = 0 ; j < good_points[i].size() ; j++)
                m.corners.push_back(good_points[i][j]);
            temp_markers.push_back(m);
        }

    }

    for(size_t i = 0; i < temp_markers.size();i++){
        Marker m = temp_markers[i];

        m.FindID(thresh);
        if(m.id != -1)
            markers.push_back(m);
    }
    if (markers.size() > 0)
    {
        vector<Point2f> preciseCorners(4 * markers.size());

        for (size_t i = 0; i<markers.size(); i++)
        {
            const vector<Point2f>& points = markers[i].corners;

            for (int c = 0; c <4; c++)
            {
                preciseCorners[i * 4 + c] = points[c];
            }
        }

        cv::TermCriteria termCriteria = cv::TermCriteria(cv::TermCriteria::MAX_ITER | cv::TermCriteria::EPS, 30, 0.01);
        cornerSubPix(thresh, preciseCorners, cvSize(5, 5), cvSize(-1, -1), termCriteria);

        // Copy refined corners position back to markers
        for (size_t i = 0; i<markers.size(); i++)
        {
            vector<Point2f>& points = markers[i].corners;

            for (int c = 0; c<4; c++)
            {
                points[c] = preciseCorners[i * 4 + c];
            }
        }
        track_flag = true;
        for(size_t i = 0 ; i < markers.size() ; i++)
            markers[i].GetTransformMatrix(camera);
    }



}

void Frame::DetectSingleMarker(Marker& marker,bool track_flag){

    vector<vector<Point> >allcontours;
    findContours(thresh, allcontours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
    vector<vector<Point> >contours;
    contours.clear();
    for (size_t i = 0; i<allcontours.size(); i++)
    {
        int contourSize = allcontours[i].size();
        if (contourSize > thresh.cols / 10)
            //if (contourSize > gray.cols/5)
        {
            contours.push_back(allcontours[i]);
        }
    }
    vector<Point> approxCurve;
    vector<vector<Point2f> > good_points;

    for (size_t i = 0; i < contours.size(); i++) {
        double eps = contours[i].size() * 0.05;
        approxPolyDP(contours[i], approxCurve, eps, true);

        if (approxCurve.size() < 4)
            continue;

        if (!isContourConvex(approxCurve))
            continue;

        float minDist = std::numeric_limits<float>::max();
        for (int i = 0; i < 4; i++)
        {
            Point side = approxCurve[i] - approxCurve[(i + 1) % 4];
            float squaredSideLength = side.dot(side);
            minDist = min(minDist, squaredSideLength);
        }

        if (minDist < 100)
            continue;

        vector<Point2f> points;

        for (int i = 0; i< 4; i++) {
            points.push_back(Point2f(approxCurve[i].x, approxCurve[i].y));
        }
        Point v1 = points[1] - points[0];
        Point v2 = points[2] - points[0];
        double cross_result = v1.x*v2.y - v2.x*v1.y;
        if (cross_result < 0)
            swap(points[1], points[3]);

        good_points.push_back(points);

    }


    vector<pair<int, int> > tooNearCandidates;
    for (size_t i = 0; i<good_points.size(); i++)
    {
        const vector<Point2f>& points1 = good_points[i];

        for (size_t j = i + 1; j<good_points.size(); j++)
        {
            const vector<Point2f>& points2 = good_points[j];

            float distSquared = 0;

            for (int c = 0; c < 4; c++)
            {
                Point v = points1[c] - points2[c];
                distSquared += v.dot(v);
            }

            distSquared /= 4;

            if (distSquared < 100)
            {
                tooNearCandidates.push_back(pair<int, int>(i, j));
            }
        }
    }

    vector<bool> removalMask(good_points.size(), false);

    for (size_t i = 0; i<tooNearCandidates.size(); i++)
    {
        float p1 = perimeter(good_points[tooNearCandidates[i].first]);
        float p2 = perimeter(good_points[tooNearCandidates[i].second]);

        size_t removalIndex;
        if (p1 > p2)
            removalIndex = tooNearCandidates[i].second;
        else
            removalIndex = tooNearCandidates[i].first;

        removalMask[removalIndex] = true;
    }
    vector<Marker > temp_markers;
    for (size_t i = 0; i<good_points.size(); i++)
    {

        if (!removalMask[i]){
            Marker m;
            for(size_t j = 0 ; j < good_points[i].size() ; j++)
                m.corners.push_back(good_points[i][j]);
            temp_markers.push_back(m);
        }

    }

    for(size_t i = 0; i < temp_markers.size();i++){
        Marker m = temp_markers[i];

        m.FindID(thresh);
        if(m.id != -1){
            marker = m;
            vector<Point2f> preciseCorners(4);
            const vector<Point2f>& points = marker.corners;
            for (int c = 0; c <4; c++)
            {
                preciseCorners[ c] = points[c];
            }
            cv::TermCriteria termCriteria = cv::TermCriteria(cv::TermCriteria::MAX_ITER | cv::TermCriteria::EPS, 30, 0.01);
            cornerSubPix(thresh, preciseCorners, cvSize(5, 5), cvSize(-1, -1), termCriteria);
            vector<Point2f>& points2f = marker.corners;

            for (int c = 0; c<4; c++)
            {
                marker.corners[c] = preciseCorners[c];
            }
            track_flag = true;
            marker.GetTransformMatrix(camera);
            return ;
        }

    }
}