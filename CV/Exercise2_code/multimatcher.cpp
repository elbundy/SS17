#include "multimatcher.h"

using namespace cv;

MultiMatcher::MultiMatcher(std::vector<cv::Mat> imageList)
{

    this->imageList = imageList;

    if(this->imageList.size() == 0){
        std::cout << "No images in MultiMatcher!" << std::endl;
        exit(-1);
    }

    // set up matrix for pairwise matches
    int num = imageList.size();
    matchMatrix.resize(num);
    for (int i = 0; i < num; ++i)
      matchMatrix[i].resize(num);

    matched = false;

    //default values for ransac
    ransac_distance = 2.0;
    ransac_conf = 0.99;

}


void MultiMatcher::extractFeatures(){

    featureList.clear();
    descriptorList.clear();

    //Detector and descriptor
    Ptr<FeatureDetector> detector = FeatureDetector::create("ORB");
    Ptr<DescriptorExtractor> extractor = DescriptorExtractor::create("ORB");

    for(int i=0; i < imageList.size(); i++){
        std::vector<cv::KeyPoint> points;
        Mat descriptors;
        detector->detect( imageList[i], points, Mat() );
        extractor->compute( imageList[i], points, descriptors );

        featureList.push_back(points);
        descriptorList.push_back(descriptors);
    }

}

void MultiMatcher::matchPairs(){


    if (descriptorList.size() == 0){
        extractFeatures();
    }

    BFMatcher matcher(cv::NORM_HAMMING, false);

    for(int i=0; i < descriptorList.size(); i++){
        for(int j=0; j < descriptorList.size(); j++){

            if( i == j){
                continue;
            }

            Mat descriptorA = descriptorList[i];
            Mat descriptorB = descriptorList[j];

            std::vector< DMatch > matches;
            matcher.match( descriptorA, descriptorB, matches );

            if (matches.size() < 1){
                std::cout << "Failed to match!" << std::endl;
            }

            std::vector< DMatch > outMatches;
            //use ransac to filter matches
            ransacTest(matches, featureList[i], featureList[j], outMatches);
            matchMatrix[i][j] = outMatches;

        }
    }

    matched = true;
}

void MultiMatcher::setRansacDistance(double val)
{
    this->ransac_distance = val;
}

void MultiMatcher::setRansacConfidence(double val)
{
    this->ransac_conf = val;
}

//Ransac
//Identify good matches using RANSAC
//https://gist.github.com/wzpan/9017475
void MultiMatcher::ransacTest(const std::vector<cv::DMatch>& matches,
                   const std::vector<cv::KeyPoint>& keypoints1,
                   const std::vector<cv::KeyPoint>& keypoints2,
                   std::vector<cv::DMatch>& outMatches) {

       double distance = ransac_distance;
       double confidence = ransac_conf;

      // Convert keypoints into Point2f
      std::vector<cv::Point2f> points1, points2;
      for (std::vector<cv::DMatch>::const_iterator it= matches.begin();
           it!= matches.end(); ++it) {

           // Get the position of left keypoints
           float x= keypoints1[it->queryIdx].pt.x;
           float y= keypoints1[it->queryIdx].pt.y;
           points1.push_back(cv::Point2f(x,y));
           // Get the position of right keypoints
           x= keypoints2[it->trainIdx].pt.x;
           y= keypoints2[it->trainIdx].pt.y;
           points2.push_back(cv::Point2f(x,y));
      }

      // Compute F matrix using RANSAC
      std::vector<uchar> inliers(points1.size(),0);
      cv::Mat fundemental= cv::findFundamentalMat(
          cv::Mat(points1),cv::Mat(points2), // matching points
          inliers,      // match status (inlier ou outlier)
          CV_FM_RANSAC, // RANSAC method
          distance,     // distance to epipolar line
          confidence);  // confidence probability

      // extract the surviving (inliers) matches
      std::vector<uchar>::const_iterator itIn= inliers.begin();
      std::vector<cv::DMatch>::const_iterator itM= matches.begin();
      // for all matches
      for ( ;itIn!= inliers.end(); ++itIn, ++itM) {

          if (*itIn) { // it is a valid match

              outMatches.push_back(*itM);
          }
    }
}

std::vector< std::vector< std::vector< cv::DMatch > > > MultiMatcher::getMatchData(){

    if(!matched){
        extractFeatures();
        matchPairs();
    }

    return matchMatrix;
}

std::vector<std::vector<KeyPoint> > MultiMatcher::getFeatureList()
{
    if (featureList.size() == 0){
        extractFeatures();
    }

    return featureList;
}

