#ifndef WRITEOFF_H
#define WRITEOFF_H

#include <opencv2/opencv.hpp>
#include <stdio.h>

using namespace cv;
using namespace std;

class WriteOff{

public:
    WriteOff();
    WriteOff(const Mat& R, const Mat& t, const std::string &filename, float ratioXY = (4.0/3.0), float scale = 1.0);

    void transform(const Mat& R, const Mat& t);

    void writeFile(const string &filename);
    void writeFile(const string &filename, const std::vector<Point3f> cam1, const std::vector<Point3f> cam2);

private:

    std::vector<Point3f> camModel;
    std::vector<Point3f> transformedModel;

    float ratioXY;
    float scale;
    Mat R;
    Mat t;
    Mat P;

    void buildTransformationMatrix(const Mat& R);
    void generateCamPoints(std::vector<Point3f> &pointVector, float ratioXY = (4.0/3.0), float scale = 1.0);
    void lineOfPoints(const Point3f start, const Point3f end, int numPoints, std::vector<Point3f> &pointVector);
};

#endif // WRITEOFF_H
