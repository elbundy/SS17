// system includes
#include <stdio.h>
#include <iostream>
#include <fstream>

// library includes
//#include <opencv2/opencv.hpp>
//#include "opencv2/core/core.hpp"

#include "cameracalibration.h"

using namespace std;
using namespace cv;

//implement a function that captures images into a given list upon keypress while displaying the video stream
void captureImages(VideoCapture cap, VideoCapture cap2, vector<Mat> &imageList, vector<Mat> &imageList2, int numImages){
    bool capture = false;
    int captured = 0;
    while(true){
        Mat frame;
        Mat frame2;
        //Doesn't work because of driver issues
        //int frameCount = videoStream.get(CV_CAP_PROP_FRAME_COUNT);
        //videoStream.set(CV_CAP_PROP_POS_FRAMES, frameCount);

        //Take current frame from webcam
        cap.read(frame);
        if(frame.empty()){
            std::cerr << "Blank frame grabbed, error!/n";
            break;
        }

        cap2.read(frame2);
        if(frame2.empty()){
            std::cerr << "Blank frame grabbed, error!/n";
            break;
        }

        //Show
        imshow("Webcam 1", frame);
        imshow("Webcam 2", frame2);

        //Wait 50ms on key press
        if(waitKey(50) >= 0)
            capture = true;

        //Capture
        if(capture){
            std::cout << "Captured photo!" << std::endl;
            imageList.push_back(frame);
            imageList2.push_back(frame2);

            captured += 1;
            capture = false;
            if(captured >= numImages){
                cv::destroyWindow("Webcam");
                break;
            }
        }
    }
}


//saves 3D points to OFF file given Mat of 3D coordinates, texture (same size) and maximum z value
//matXYZ has to be of type cv::Mat_<cv::Vec3f> containing X,Y,Z coordinates
static void writeOFF(const char* filename, const Mat& matXYZ, const Mat color, double max_z = 200.0)
{
    ofstream fp (filename);

    int numPoints = 0;

    for(int y = 0; y < matXYZ.rows; y++)
    {
        for(int x = 0; x < matXYZ.cols; x++)
        {
            Vec3f point = matXYZ.at<Vec3f>(y, x);
            if(fabs(point[2] - max_z) < FLT_EPSILON || fabs(point[2]) > max_z){ 
                continue;
            }
            numPoints++;
        }
    }

    fp << "COFF\n";
    fp << numPoints <<" 0 0\n";

    for(int y = 0; y < matXYZ.rows; y++)
    {
        for(int x = 0; x < matXYZ.cols; x++)
        {
            Vec3f point = matXYZ.at<Vec3f>(y, x);
            if(fabs(point[2] - max_z) < FLT_EPSILON || fabs(point[2]) > max_z){ 
                continue;
            }

            Vec3b intensity = color.at<Vec3b>(y, x);
            uchar blue = intensity.val[0];
            uchar green = intensity.val[1];
            uchar red = intensity.val[2];

            fp << point[0] << " "<< point[1] << " "<< point[2] << " " << int(red) << " "<< int(green) << " "<< int(blue) << "\n";
        }
    }

    fp.close();
}



/*********************************************************************************************/
//calculate 3D points given disparity and matrix Q (returned by stereoRectify)
static cv::Mat_<cv::Vec3f> calcXYZ(const Mat disparity32F, const Mat Q)
{
    cv::Mat_<cv::Vec3f> XYZ(disparity32F.size());   // Output point cloud

    std::cout << disparity32F.size() << std::endl;
    //calculate xyz coordinates
    for(int row=0; row<disparity32F.rows; row++){
        for(int col=0; col<disparity32F.cols; col++){
            if(disparity32F.at<float>(row, col) <= 0.f){
                Vec3f sol(0, 0, 100000);
                XYZ(row, col) = sol;
                continue;
            }
            //Q \cdot (x, y, disparity, 1)^T = (X, Y, Z, W)^T 
            Mat_<double> vec(4,1);
            vec(0,0) = row; //x
            vec(1,0) = col; //y
            vec(2,0) = disparity32F.at<float>(row,col);
            vec(3,0) = 1; 
            Mat_<double> mult = Q * vec;

            //3-dim coords: (X/W, Y/W, Z/W)
            Vec3f sol((float)(mult(0,0)/mult(3,0)), mult(1,0)/mult(3,0), mult(2,0)/mult(3,0));
            std::cout << sol << std::endl;
            XYZ(row, col) = sol;
        }
    }
    //reprojectImageTo3D(disparity32F, XYZ, Q);
    //std::cout << XYZ << std::endl;
    return XYZ;
}

int main(int argc, char** argv )
{
    bool calibrate = false;
    if(argc < 2){
    }
    else if(strcmp(argv[1], "--calibrate") == 0 || strcmp(argv[1], "-c") == 0){
        calibrate = true;
    }   

    //Parameters we might need
    Mat R, T, E, F;
    Mat R1, R2, P1, P2, Q;
    Rect validRoi1;
    Rect validRoi2;

    Mat cameraMatrix1;
    Mat distCoeffs1;
    Mat cameraMatrix2;
    Mat distCoeffs2;

    vector<Mat> imageList;
    vector<Mat> imageList2;

    if(calibrate){
        /**********************************************************************/
        /************************Stereo Calibration****************************/
        /**********************************************************************/
        //Open two video captures
        //The FPS might have to be set to a low value (e.g. 10) because of USB bandwidth
        cv::VideoCapture cap = VideoCapture(0);
        if(!cap.isOpened())
            return -1;
        cap.set(CV_CAP_PROP_FPS, 10);


        cv::VideoCapture cap2 = VideoCapture(1);
        if(!cap2.isOpened())
            return -1;
        cap2.set(CV_CAP_PROP_FPS, 10);


        //Capture images from both streams simultanously and save into list
        int numImages = 10;
        captureImages(cap, cap2, imageList, imageList2, numImages);
        std::cout << imageList.size() << " " << imageList2.size() << std::endl;

        //Set boardsize and squaresize of calibration pattern
        Size boardSize = Size(9,7);
        float squareSize = 0.0025;

        //Use the images in your list from the video stream for intrinsic calibration of each camera
        //feel free to reuse code from Ex4 / the opencv tutorial
        cameraMatrix1 = Mat::eye(3, 3, CV_64F);
        distCoeffs1 = Mat::zeros(8, 1, CV_64F);
        cameraMatrix2 = Mat::eye(3, 3, CV_64F);
        distCoeffs2 = Mat::zeros(8, 1, CV_64F);

        //Calibrate
        CameraCalibration calibration(imageList, boardSize, squareSize);
        calibration.calibrate(cameraMatrix1, distCoeffs1);

        CameraCalibration calibration2(imageList2, boardSize, squareSize);
        calibration2.calibrate(cameraMatrix2, distCoeffs2);

        cout << "cameraMatrix 1: " << endl << cameraMatrix1 << endl;
        cout << "distCoeffs 1: "<< endl << distCoeffs1 << endl;
        cout << "cameraMatrix 2: " << endl << cameraMatrix2 << endl;
        cout << "distCoeffs 2: "<< endl << distCoeffs2 << endl;

        //get the object points as well as the detected image points of both cameras
        vector<vector<Point3f> > objectPoints(calibration2.objectPoints); 
        vector<vector<Point2f> > imagePoints1(calibration.imagePoints);
        vector<vector<Point2f> > imagePoints2(calibration2.imagePoints);
        //imagePoints1 = vector<vector<Point3f>>(calibration.imagePoints);

        if (imagePoints1.size() != imagePoints2.size()){
            cout << "Not same number of imagePoints!" << endl;
        }

        //use the imagepoints, objectpoints and intrinsic calibration as input for the stereo calibration
        //set at least the CALIB_FIX_INTRINSIC flag to use your prior estimation of instrinsic paramters
        std::cout << " 1 " << std::endl;
        double reprojectionError = stereoCalibrate(objectPoints, imagePoints1, imagePoints2, cameraMatrix1, distCoeffs1, cameraMatrix2, distCoeffs2, imageList[0].size(), R, T, E, F, TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 1e-6), CALIB_FIX_INTRINSIC);

        //estimate the parameters for stereo rectification

        stereoRectify(cameraMatrix1, distCoeffs1, cameraMatrix2, distCoeffs2, imageList[0].size(), R, T, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY, -1, imageList[0].size(), &validRoi1, &validRoi2 );
        std::cout << " 2 " << std::endl;

        //save your estimated parameters
        //OpenCV FileStorage is a good option
        //implement a setting to change between starting a new calibration and loading your previously saved parameters
        //setting can be hardcoded
        cv::FileStorage storage("params.yml", cv::FileStorage::WRITE);
        storage << "camMat1" << cameraMatrix1; 
        storage << "camMat2" << cameraMatrix2; 
        storage << "imgList1" << imageList;
        storage << "imgList2" << imageList2;
        storage << "dist1" << distCoeffs1; 
        storage << "dist2" << distCoeffs2; 
        storage << "R" << R; 
        storage << "T" << T; 
        storage << "E" << E;
        storage << "F" << F;
        storage << "R1" << R1;
        storage << "R2" << R2;
        storage << "P1" << P1;
        storage << "P2" << P2;
        storage << "Q" << Q;
        storage << "roi1" << validRoi1;
        storage << "roi2" << validRoi2;
        storage.release();  
    }
    else{
        //Read from memory
        cv::FileStorage storage("params.yml", cv::FileStorage::READ);
        storage["camMat1"] >> cameraMatrix1; 
        storage["camMat2"] >> cameraMatrix2; 
        storage["imgList1"] >> imageList;
        storage["imgList2"] >> imageList2;
        storage["dist1"] >> distCoeffs1; 
        storage["dist2"] >> distCoeffs2; 
        storage["R"] >> R; 
        storage["T"] >> T; 
        storage["E"] >> E;
        storage["F"] >> F;
        storage["R1"] >> R1;
        storage["R2"] >> R2;
        storage["P1"] >> P1;
        storage["P2"] >> P2;
        storage["Q"] >> Q;
        storage["roi1"] >> validRoi1;
        storage["roi2"] >> validRoi2;
        storage.release(); 
    }


    /**********************************************************************/
    /*********Stereo Matching and 3D Point Cloud reconstruction************/
    /**********************************************************************/
    //load your parameters or use the just estimated ones (depending on setting)
    //depending on your implementation you might want to refactor your code ;)
    //Already done

    //compute the mapping for stereo rectification given the estimated paramaters
    Mat rmap[2][2];
    initUndistortRectifyMap(cameraMatrix1, distCoeffs1, R1, P1, imageList[0].size(), CV_32FC1, rmap[0][0], rmap[0][1]);
    initUndistortRectifyMap(cameraMatrix2, distCoeffs2, R2, P2, imageList2[0].size(), CV_32FC1, rmap[1][0], rmap[1][1]);

    //Rectified images
    Mat rimg, rimg2;
    //Distorted, nonrectified images
    Mat frame, frame2;
    //Images in which we will save our disparities
    Mat imgDisparity32F;
    Mat imgDisparity8U;

    //setup stereo blockmatcher
    //numDisp, sadwindowsize
    //the BM computes a matching function based on a window (sum of absolute distance)
    //the larger, the fewer false matches
    //assumption: disparity is the same over the area of the window
    //the corresponding point of a point (x,y) in the left image must be at the same location (x,y) in the right image or to the left side of it
    //minDispartity tells BM how much to the left the point must at least be
    //minDisparity + numberOfDisparities = maxDisparity
    //Prefiltering is used to normalize occlusion, lighting, etc.
    //sbm.state->SADWindowSize = 9;
    //sbm.state->numberOfDisparities = 112; //gets set in constructor
    //sbm.state->textureThreshold = 507; //minimal amount of texture needed
    //sbm.state->uniquenessRatio = 0;
    //sbm.state->speckleWindowSize = 0;
    //sbm.state->speckleRange = 8;
    //sbm.state->disp12MaxDiff = 1;

    //experiment with your stereo block matching settings
    //especially: SADWindowSize and numberOfDisparities
    int ndisparities = 16*10; //has to be multiple of 16 
    int SADWindowSize = 19; // must be odd
    StereoBM sbm(0, ndisparities, SADWindowSize); //Preset: BASIC_PRESET
    //set minDisparity
    sbm.state->minDisparity = 0; 
    //set valid rois in block matcher
    sbm.state->roi1 = validRoi1;
    sbm.state->roi2 = validRoi2;


    sbm.state->preFilterSize = 5;
    sbm.state->preFilterCap = 61;

    //Initialize VideoCaptures
    cv::VideoCapture cap = VideoCapture(0);
    if(!cap.isOpened())
        return -1;
    cap.set(CV_CAP_PROP_FPS, 10);


    cv::VideoCapture cap2 = VideoCapture(1);
    if(!cap2.isOpened())
        return -1;
    cap2.set(CV_CAP_PROP_FPS, 10);

    //show your rectified video streams
    //estimate the disparity using the previously setup StereoBM and show the result
    int keyPress = -1;
    while(true){
        //Take current frame from webcams
        cap.read(frame);
        if(frame.empty()){
            std::cerr << "Blank frame grabbed, error!/n";
            break;
        }
        cap2.read(frame2);
        if(frame2.empty()){
            std::cerr << "Blank frame grabbed, error!/n";
            break;
        }

        //Show
        imshow("Webcam 1", frame);
        imshow("Webcam 2", frame2);

        //stereo rectify the images using the previously computed mappings
        remap(frame, rimg, rmap[0][0], rmap[0][1], INTER_NEAREST, BORDER_TRANSPARENT, Scalar());
        remap(frame2, rimg2, rmap[1][0], rmap[1][1], INTER_NEAREST, BORDER_TRANSPARENT, Scalar());
        //Show
        imshow("Webcam 1 rect", rimg);
        imshow("Webcam 2 rect", rimg2);

        //compute the disparity image using the stereo block matcher
        //use CV_32F as output
        Mat rimg_g, rimg2_g;
        rimg.convertTo(rimg_g, CV_8U);
        cvtColor(rimg_g, rimg_g, CV_BGR2GRAY);
        rimg2.convertTo(rimg2_g, CV_8U);
        cvtColor(rimg2_g, rimg2_g, CV_BGR2GRAY);
        sbm(rimg_g, rimg2_g, imgDisparity32F, CV_32F);


        //Check for minimum and maximum disparity values
        double minVal, maxVal;
        minMaxLoc(imgDisparity32F, &minVal, &maxVal);

        //convert disparity to CV_8UC1 with range 0-255 to display your resulting disparity image
        imgDisparity32F.convertTo(imgDisparity8U, CV_8UC1, 255/(maxVal-minVal));
        imshow("Disparity", imgDisparity8U);
        //std::cout << imgDisparity32F << std::endl;

        if(waitKey(10) >= 0)
            keyPress = 32;

        //on keypress calculate the XYZ coordinates given the disparity
        //calculate the XYZ using the matrix Q
        //write the results into an OFF file (use the writeOFF function) show the result in meshlab
        if(keyPress == 32){
            Mat xyz;
            //reproject the disparity image into 3D space
            xyz = calcXYZ(imgDisparity32F, Q);

            //saves 3D points to OFF file given Mat of 3D coordinates, texture/color (same size) and maximum depth value
            //matXYZ has to be of type cv::Mat_<cv::Vec3f> containing X,Y,Z coordinates
            writeOFF("point_cloud.off", xyz, rimg, 10000);
            break;
        }

    }
    return 0;
}
