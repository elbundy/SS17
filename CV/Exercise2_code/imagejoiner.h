#ifndef IMAGEJOINER_H
#define IMAGEJOINER_H

#include <opencv2/opencv.hpp>
#include <stdio.h>

//Image joiner class
//estimates transformations between images and blends them to create a panography

//Feel free to change the interface!
class ImageJoiner{
public:
    //constructor for the image joiner class
    //receives a list of images, will call the multimatcher internally
    ImageJoiner(std::vector<cv::Mat> imageList);

    //calculate the transformations between overlapping images
    void transformPairwise();

    //join the images to creata a panography like output
    cv::Mat joinImages();


private:
    //data structures

    //matrix containing matches between all image pairs
    std::vector< std::vector< std::vector< cv::DMatch > > > matchMatrix;
    //matrix containing the features of all images
    std::vector<std::vector<cv::KeyPoint> > featureList;

    //list of images
    std::vector<cv::Mat> imageList;

    //TODO
    //add required members

};

#endif //IMAGEJOINER_H
