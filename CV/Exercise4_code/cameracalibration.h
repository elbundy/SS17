#ifndef CAMERACALIBRATION_H
#define CAMERACALIBRATION_H

// system includes
#include <stdio.h>
// library includes
#include <opencv2/opencv.hpp>
#include "opencv2/core/core.hpp"

using namespace std;
using namespace cv;

//The CameraCalibration class will estimate a cameraMatrix and distortion coefficients
//input: list of images, size of calibration patterns

//TODO
//implement class - feel free to add/change
//more comments in .cpp
class CameraCalibration
{
public:
    CameraCalibration(const vector<Mat> imageList, Size boardSize, float squareSize);

    void setImageList(const vector<Mat> imageList);
    void calibrate(Mat &cameraMatrix,  Mat &distCoeffs);
    void showImage(int num);

private:

    void findImagePoints();
    void generateObjectPoints();

    vector<Mat> imageList;

    Size boardSize;
    float squareSize;

    vector<vector<Point2f> > imagePoints;
    vector<vector<Point3f> > objectPoints;

    Mat cameraMatrix;
    Mat distCoeffs;
    vector<Mat> rvecs;
    vector<Mat> tvecs;

};

#endif // CAMERACALIBRATION_H
