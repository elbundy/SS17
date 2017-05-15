#ifndef CORNERDETECTOR_H
#define CORNERDETECTOR_H

#include <opencv2/opencv.hpp>
#include <stdio.h>

class CornerDetector{
public:
    //TODO add whats needed
    //feel free to change the interface or define new functions

    CornerDetector();
    CornerDetector(cv::Mat &img, int blockSize);

    std::vector<cv::KeyPoint> detectFeatures();



private:
    //TODO add whats needed

    std::vector<cv::KeyPoint> keyPoints;
    cv::Mat img;
    int blockSize;
};

#endif //CORNERDETECTOR_H
