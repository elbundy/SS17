#ifndef CORNERDETECTOR_H
#define CORNERDETECTOR_H

#include <opencv2/opencv.hpp>
#include <stdio.h>

class CornerDetector{
public:
    CornerDetector();
    CornerDetector(cv::Mat &img, int blockSize);
    std::vector<cv::KeyPoint> detectFeatures();

private:
    std::vector<cv::KeyPoint> keyPoints;
    cv::Mat img;
    int blockSize;
};

#endif //CORNERDETECTOR_H
