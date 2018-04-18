#include "Marker.h"



int Marker::hammDistMarker18(Mat bits)
{
    int ids[8][6] =
            {
                    { 1,0,0,0,0,0 },
                    { 0,1,0,1,0,1 },
                    { 1,0,0,1,1,0 },
                    { 1,1,0,0,1,1 },
                    { 1,1,1,0,0,0 },
                    { 1,0,1,1,0,1 },
                    { 0,1,1,1,1,0 },
                    { 0,0,1,0,1,1 }
            };

    int dist = 0;

    for (int y = 0; y<6; y++)
    {
        int minSum = 1e5; //hamming distance to each possible word

    for (int p = 0; p<8; p++)
    {
        int sum = 0;
    //now, count
        for (int x = 0; x<6; x++)
        {
        sum += bits.at<uchar>(y, x) == ids[p][x] ? 0 : 1;
        }

        if (minSum>sum)
            minSum = sum;
        }

        //do the and
        dist += minSum;
    }

    return dist;
}

Mat Marker::rotate(Mat in)
{
    cv::Mat out;
    in.copyTo(out);
    for (int i = 0; i<in.rows; i++)
    {
        for (int j = 0; j<in.cols; j++)
        {
            out.at<uchar>(i, j) = in.at<uchar>(in.cols - j - 1, i);
        }
    }
return out;
}


int Marker::mat2id18(const Mat &bits)
{
    int val = 0;
    for (int y = 0; y<6; y++)
    {
        val <<= 1;
        if (bits.at<uchar>(y, 2)) val |= 1;
        val <<= 1;
        if (bits.at<uchar>(y, 4)) val |= 1;
        val <<= 1;
        if (bits.at<uchar>(y, 5)) val |= 1;
    }
    return val;
}

void Marker::FindID(Mat thresh){

    vector<Point2f> marker_corners2d(4);
    marker_corners2d[0] = Point2f(0, 0);
    marker_corners2d[1] = Point2f(99, 0);
    marker_corners2d[2] = Point2f(99, 99);
    marker_corners2d[3] = Point2f(0, 99);

    Mat after_perspective;
    vector<Point2f>& points = corners;

    Mat markerTransform = getPerspectiveTransform(points, marker_corners2d);

    warpPerspective(thresh, after_perspective, markerTransform, Size(100, 100));

    int nRotations;

    threshold(after_perspective, after_perspective, 125, 255, THRESH_BINARY_INV | THRESH_OTSU);

    int cellSize = after_perspective.rows / 8;

    for (int y = 0; y<8; y++)
    {
        int inc = 7;

        if (y == 0 || y == 7) inc = 1; //for first and last row, check the whole border

        for (int x = 0; x<8; x += inc)
        {
            int cellX = x * cellSize;
            int cellY = y * cellSize;
            Mat cell = after_perspective(Rect(cellX, cellY, cellSize, cellSize));

            int nZ = cv::countNonZero(cell);

            if (nZ >(cellSize*cellSize) / 2)
            {
                continue;
            }
        }
    }
    Mat bitMatrix = Mat::zeros(6, 6, CV_8UC1);

    for (int y = 0; y<6; y++)
    {
        for (int x = 0; x<6; x++)
        {
            int cellX = (x + 1)*cellSize;
            int cellY = (y + 1)*cellSize;
            Mat cell = after_perspective(cv::Rect(cellX, cellY, cellSize, cellSize));

            int nZ = cv::countNonZero(cell);
            if (nZ>(cellSize*cellSize) / 2)
            bitMatrix.at<uchar>(y, x) = 1;
        }
    }

    Mat rotations[4];
    int distances[4];

    rotations[0] = bitMatrix;
    distances[0] = hammDistMarker18(rotations[0]);

    pair<int, int> minDist(distances[0], 0);

    for (int i = 1; i<4; i++)
    {
        rotations[i] = rotate(rotations[i - 1]);//˳ʱ֫ת90
        distances[i] = hammDistMarker18(rotations[i]);

        if (distances[i] < minDist.first)
        {
            minDist.first = distances[i];
            minDist.second = i;
        }
    }

    nRotations = minDist.second;
    if (minDist.first == 0)
    {
        id = mat2id18(rotations[minDist.second]);
    }

    if (id != -1)
    {
        std::rotate(points.begin(), points.begin() + 4 - nRotations, points.end());
    }


}

void Marker::GetTransformMatrix(Camera camera){
    vector<Point3f> marker_corner3d(4);
//    marker_corner3d[0] = Point3f(-0.5f, -0.5f, 0);
//    marker_corner3d[1] = Point3f(-0.5f, +0.5f, 0);
//    marker_corner3d[2] = Point3f(+0.5f, +0.5f, 0);
//    marker_corner3d[3] = Point3f(+0.5f, -0.5f, 0);
    marker_corner3d[0] = Point3f(-0.5f, -0.5f, 0);
    marker_corner3d[1] = Point3f(0.5f, -0.5f, 0);
    marker_corner3d[2] = Point3f(0.5f, +0.5f, 0);
    marker_corner3d[3] = Point3f(-0.5f, 0.5f, 0);


    cv::Mat Rvec;
    cv::Mat_<float> Tvec;
    cv::Mat raux, taux;
    cv::solvePnP(marker_corner3d, corners, camera.camMatrix, camera.distCoeffs, raux, taux);
    raux.convertTo(Rvec, CV_32F);
    taux.convertTo(Tvec, CV_32F);


    cv::Mat_<float> rotMat(3, 3);
    cv::Rodrigues(Rvec, rotMat);
    world2camera = (Mat_ <float>(4, 4) <<
                               rotMat.at<float>(0, 0), rotMat.at<float>(0, 1), rotMat.at<float>(0, 2), Tvec.at<float>(0),
            rotMat.at<float>(1, 0), rotMat.at<float>(1, 1), rotMat.at<float>(1, 2), Tvec.at<float>(1),
            rotMat.at<float>(2, 0), rotMat.at<float>(2, 1), rotMat.at<float>(2, 2), Tvec.at<float>(2),
            0, 0, 0, 1);


    Mat camera2world_rotation;
    invert(rotMat, camera2world_rotation);
    camera2world = (Mat_ <float>(4, 4) <<
                                  camera2world_rotation.at<float>(0, 0), camera2world_rotation.at<float>(0, 1), camera2world_rotation.at<float>(0, 2), -Tvec.at<float>(0),
            camera2world_rotation.at<float>(1, 0), camera2world_rotation.at<float>(1, 1), camera2world_rotation.at<float>(1, 2), -Tvec.at<float>(1),
            camera2world_rotation.at<float>(2, 0), camera2world_rotation.at<float>(2, 1), camera2world_rotation.at<float>(2, 2), -Tvec.at<float>(2),
            0, 0, 0, 1);

    vector<Mat> axis(4);
    vector<Mat> axis_transformed(4);
    axis_points.resize(4);
    axis[0] = (Mat_ <float>(4, 1) << 0, 0, 0, 1);
    axis[1] = (Mat_ <float>(4, 1) << 0.5, 0, 0, 1);
    axis[2] = (Mat_ <float>(4, 1) << 0, 0.5, 0, 1);
    axis[3] = (Mat_ <float>(4, 1) << 0, 0, 0.5, 1);
    for (int number = 0; number < 4; number++) {
        axis_transformed[number] = world2camera * axis[number];
        for (int row = 0; row < 3; row++) {
            axis_transformed[number].at<float>(row, 0) /= axis_transformed[number].at<float>(2, 0);
        }
        axis_points[number].x = axis_transformed[number].at<float>(0, 0) * camera.camMatrix.at<float>(0, 0) + camera.camMatrix.at<float>(0, 2);
        axis_points[number].y = axis_transformed[number].at<float>(1, 0) * camera.camMatrix.at<float>(0, 0) + camera.camMatrix.at<float>(0, 2);
    }


}