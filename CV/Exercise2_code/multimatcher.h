#ifndef MULTIMATCHER_H
#define MULTIMATCHER_H

#include <opencv2/opencv.hpp>
#include <stdio.h>

class MultiMatcher{
public:

    MultiMatcher(std::vector<cv::Mat> imageList);

    void setRansacDistance(double val);
    void setRansacConfidence(double val);

    void matchPairs();
    void extractFeatures();

    std::vector< std::vector< std::vector< cv::DMatch > > > getMatchData();
    std::vector<std::vector<cv::KeyPoint> > getFeatureList();

private:

    void ransacTest(const std::vector<cv::DMatch> &matches, const std::vector<cv::KeyPoint> &keypoints1, const std::vector<cv::KeyPoint> &keypoints2, std::vector<cv::DMatch> &outMatches);

    std::vector<cv::Mat> imageList;
    std::vector<cv::Mat> descriptorList;
    std::vector< std::vector< std::vector< cv::DMatch > > > matchMatrix;
    std::vector<std::vector<cv::KeyPoint> > featureList;

    double ransac_distance;
    double ransac_conf;

    bool matched;

};

#endif //MULTIMATCHER_H
