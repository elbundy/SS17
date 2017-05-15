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

int main(int argc, char** argv )
{

    //use cmdl line (if wanted)
    /*if ( argc != 3 )
    {
        printf("usage: main <Image_Path1> <Image_Path2>\n");
        return -1;
    }*/


    //load two images for feature matching
    Mat img;
    img = imread("house.jpg", CV_LOAD_IMAGE_COLOR);

    //Show image
    namedWindow("Image", WINDOW_AUTOSIZE);
    imshow("Image", img);
    waitKey(0);

    //preprocess (resize...) if needed
    //TODO


    //Feature Detection
    //detect features in your images
    //use your own implementation of a corner detector
    CornerDetector c(img, 7);

    std::vector<cv::KeyPoint> keyPoints;
    keyPoints = c.detectFeatures();

    //Visualize the detected keypoints
    for(int i=0; i<keyPoints.size(); i++){
        circle(img, keyPoints[i].pt, 10, Scalar(255.0, 0.0, 0.0));
    }

    //Show image
    namedWindow("Image", WINDOW_AUTOSIZE);
    imshow("Image", img);
    waitKey(0);



    //Feature description (ORB,BRIEF,BRISK,FREAK...)
    //use OpenCV to extract feature descriptions for your feature points
    //feel free to use different available descriptors (read about them in the documentation)
    Ptr<DescriptorExtractor> extractor = DescriptorExtractor::create("ORB");
    //TODO


    //Feature matching (the norm should fit your descriptor (see documentation!))
    BFMatcher matcher(cv::NORM_HAMMING, false);
    //TODO


    //find good matches
    //think about a strategy to filter out the bad matches
    //TODO

    //visualize the good matches
    //TODO

    return 0;
}
