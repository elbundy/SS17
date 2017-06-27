// system includes
#include <stdio.h>
#include <iostream>
#include <fstream>

// library includes
#include <opencv2/opencv.hpp>
#include "opencv2/core/core.hpp"

#include "cameracalibration.h"

using namespace std;
using namespace cv;


//implement a function that captures images into a given list upon keypress while displaying the video stream
void captureImages( VideoCapture videoStream, vector<Mat> &imageList, int numImages){
    bool capture = false;
    int captured = 0;
    while(true){
        Mat frame;
        //Doesn't work because of driver issues
        //int frameCount = videoStream.get(CV_CAP_PROP_FRAME_COUNT);
        //videoStream.set(CV_CAP_PROP_POS_FRAMES, frameCount);

        //Take current frame from webcam
        videoStream.read(frame);
        if(frame.empty()){
            std::cerr << "Blank frame grabbed, error!/n";
            break;
        }

        //Show
        imshow("Webcam", frame);

        //Wait 50ms on key press
        if(waitKey(50) >= 0)
            capture = true;
        //Capture
        if(capture){
            imageList.push_back(frame);
            captured += 1;
            if(captured >= numImages){
                break;
            }
        }
    }
}


int main(int argc, char** argv )
{
    //open the videostream of a single webcam (VideoCapture)
    //set resolution and fps settings for the stream
    cv::VideoCapture cap = VideoCapture(0);
    if(!cap.isOpened())
        return -1;

    //implement the function captureImages()
    //pass your VideoCapture, empty imageList and how many images should be taken
    //it will show the video stream and capture images
    //for the calibration experiment with the number of images
    //they should display the chessboard in a varity of views
    int numberOfImages = 10;
    vector<Mat> imageList;
    captureImages(cap, imageList , numberOfImages);

    //check if at least one images was captured
    if (imageList.size() < 1){
        cout << "Not enough images!" << endl;
        return -1;
    }
    //check if exact amount of pictures captured
    if(imageList.size() != numberOfImages){
        cerr << "Requested amount of images not met!" << endl;
        return -1;
    }

    //set boardsize and squaresize according to your calibration pattern
    Size boardSize = Size(9,7);
    float squareSize = 25.0;

    Mat cameraMatrix = Mat::eye(3, 3, CV_64F);
    Mat distCoeffs = Mat::zeros(8, 1, CV_64F);

    //TODO
    //implement and call the CameraCalibration class
    //it will return a cameraMatrix and distortion coefficients
    CameraCalibration calibration(imageList, boardSize, squareSize);

    cout << "cameraMatrix: "<< cameraMatrix << endl;
    cout << "distCoeffs: "<< distCoeffs << endl;

    //TODO
    //use the camera matrix and distortion coefficients to undistort your camera stream
    //show the original and undistorted stream for qualitative comparision of the estimated parameters


    return 0;
}
