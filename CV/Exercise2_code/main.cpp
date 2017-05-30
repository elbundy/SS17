// system includes
#include <stdio.h>
#include <iostream>

// library includes
#include <opencv2/opencv.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"

#include "multimatcher.h"
#include "imagejoiner.h"

using namespace std;
using namespace cv;

int main(int argc, char** argv )
{
    //load list of images (hardcoded or parse folder...)
    //feel free to take some overlapping images yourself
    String folderpath = "../Exercise2_img/*.jpg";
    std::vector<String> filenames;
    cv::glob(folderpath, filenames);

    //create list of images to pass to your image joiner
    std::vector<cv::Mat> imageList;

    for (size_t i=0; i<filenames.size(); i++){
        Mat im = imread(filenames[i]);
        imageList.push_back(im);
    }        


    for(int i=0; i<imageList.size(); i++){
        //check if all images are loaded correctly
        if (!imageList[i].data){
            cout <<  "Could not load all images!" << std::endl ;
            return -1;
        }

        //TODO
        //preprocess (e.g. resize) your images if needed
        //if you use your own images you might want to resize them etc.
        //you might have to play around a bit with settings to get enough good matches for the transformation estimation
    }

    //TODO
    //implement the imageJoiner class!
    //call your image joiner to calculate the transforms between the image pairs and join the images into a single canvas
    //create image joiner instance
    ImageJoiner imageJoiner(imageList);
    //calculate transforms between image pairs
    imageJoiner.transformPairwise();
    //create panography image
    Mat result = imageJoiner.joinImages();

    //show result
    imshow( "Result", result );
    moveWindow("Result", 150, 150);

    waitKey(0);

    return 0;
}
