// system includes
#include <stdio.h>
#include <iostream>

// library includes
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

// own includes
#include "cornerdetector.h"

using namespace cv;

//Function for sorting DMatch vector in ascending order by value of distance
bool compareDMatch(DMatch a, DMatch b){ return a.distance<b.distance; }

int main(int argc, char** argv )
{
    //use cmdl line (if wanted)
    if ( argc != 3 )
    {
        printf("usage: main <Image_Path1> <Image_Path2>\n");
        return -1;
    }

    //1) load two images for feature matching
    Mat img1, img2;
    img1 = imread(argv[1], CV_LOAD_IMAGE_COLOR);
    img2 = imread(argv[2], CV_LOAD_IMAGE_COLOR);

    Size sz1 = img1.size();
    Size sz2 = img2.size();

    //2) Show images
    Mat imgFused(sz1.height, sz1.width + sz2.width, CV_8UC3);
    img1.copyTo(imgFused(Rect(0, 0, sz1.width, sz1.height)));
    img2.copyTo(imgFused(Rect(sz1.width, 0, sz2.width, sz2.height)));
    namedWindow("Images", CV_WINDOW_NORMAL);
    setWindowProperty("Images", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
    imshow("Images", imgFused);
    waitKey(0);

    //3) preprocess (resize...) if needed
    //Maybe when doing multiscale approach?

    //4) Feature Detection
    //detect features in your images
    //use your own implementation of a corner detector
    CornerDetector c1(img1, 5);
    CornerDetector c2(img2, 5);

    std::vector<cv::KeyPoint> keyPoints1, keyPoints2;
    keyPoints1 = c1.detectFeatures();
    keyPoints2 = c2.detectFeatures();

    //5) Visualize the detected keypoints
    Mat img1Keypoints;
    Mat img2Keypoints;
    drawKeypoints(img1, keyPoints1, img1Keypoints, Scalar(255.0, 0.0, 0.0), 0);
    drawKeypoints(img2, keyPoints2, img2Keypoints, Scalar(255.0, 0.0, 0.0), 0);
    Mat imgKeypointsFused(sz1.height, sz1.width + sz2.width, CV_8UC3);
    img1Keypoints.copyTo(imgKeypointsFused(Rect(0, 0, sz1.width, sz1.height)));
    img2Keypoints.copyTo(imgKeypointsFused(Rect(sz1.width, 0, sz2.width, sz2.height)));
    namedWindow("Keypoints", CV_WINDOW_NORMAL);
    setWindowProperty("Keypoints", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
    imshow("Keypoints", imgKeypointsFused);
    waitKey(0);

    //6) Feature description (ORB,BRIEF,BRISK,FREAK...)
    //use OpenCV to extract feature descriptions for your feature points
    //feel free to use different available descriptors (read about them in the documentation)
    Ptr<DescriptorExtractor> extractor = DescriptorExtractor::create("ORB");
    //Compute descriptors
    Mat desc1, desc2;
    extractor->compute(img1, keyPoints1, desc1);
    extractor->compute(img2, keyPoints2, desc2);

    //7) Feature matching (the norm should fit your descriptor (see documentation!))
    //NORM_HAMMING is appropriate for ORB descriptors
    BFMatcher matcher(cv::NORM_HAMMING, false);
    vector<vector<DMatch>> matches;

    //For every descriptor find 2 best matches (for ratio test)
    matcher.knnMatch(desc1, desc2, matches, 2);

    //find good matches via ratio test (from SIFT paper)
    vector<DMatch> goodMatches;
    for(int i=0; i<matches.size(); i++){
        const float ratio = 0.8;
        if(matches[i][0].distance < ratio * matches[i][1].distance){
            goodMatches.push_back(matches[i][0]);
        }
    }
    
    //8) Visualize 100 best matches (or less if there aren't enough)
    sort(goodMatches.begin(), goodMatches.end(), compareDMatch);
    if(goodMatches.size() > 100){
        goodMatches.resize(100);
    }
    Mat matchImg;
    drawMatches(img1, keyPoints1, img2, keyPoints2, goodMatches, matchImg);

    namedWindow("Image", CV_WINDOW_NORMAL);
    setWindowProperty("Image", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
    imshow("Image", matchImg);
    waitKey(0);
    return 0;
}
