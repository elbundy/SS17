#include "cameracalibration.h"

//constructor setting given parameters and default values
CameraCalibration::CameraCalibration(const vector<Mat> imageList, Size boardSize, float squareSize): imageList{imageList}, boardSize{boardSize}, squareSize{squareSize}
{

}

//for reuse set new imageList, clear previous data
void CameraCalibration::setImageList(const vector<Mat> imageList)
{
    this->imageList = imageList;
    this->imagePoints = vector<vector<Point2f>>();
    this->objectPoints = vector<vector<Point3f>>();
}

//TODO
//this function will return cameraMatrix and distortion coefficients
void CameraCalibration::calibrate(Mat &cameraMatrix, Mat &distCoeffs)
{

    //TODO
    //implement functions finding the chessboard corners in the images and generating the object corners according to board parameters
    findImagePoints();
    generateObjectPoints();

    //TODO
    //Find intrinsic and extrinsic camera parameters - calibrateCamera

    //TODO
    //print returned re-projection error
    cout << "Re-projection error: " << endl;

    cameraMatrix = this->cameraMatrix.clone();
    distCoeffs = this->distCoeffs.clone();

}

//find the chessboard corners in the calibration images
//draw the detected corners onto the images
//the final image points should be pushed into this->imagePoints
void CameraCalibration::findImagePoints()
{
    for(std::vector<Mat>::iterator it = imageList.begin(); it < imageList.end(); it++){
        //Image to grayscale
        Mat gray;
        cvtColor(*it, gray, CV_BGR2GRAY); 

        //Find inner corners of chessboard
        vector<Point2f> imagePoints;
        bool found = findChessboardCorners(gray, boardSize, imagePoints, CV_CALIB_CB_ADAPTIVE_THRESH );
        if(found){
            //Refine corner coordinates
            cornerSubPix(gray, imagePoints, Size(11, 11), Size(-1, -1),
                TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

            //Draw corners
            drawChessboardCorners(*it, boardSize, imagePoints, found);

            //Append found points to member vector
            this-> imagePoints = imagePoints;
        }
    }
    

}

//TODO
//generate the object points of the calibration pattern
//use boardSize and squareSize
//they should be saved into this->objectPoints
void CameraCalibration::generateObjectPoints()
{
    //Worlds origin is top left corner point
    for(std::vector<Mat>::iterator it = imageList.begin(); it < imageList.end(); it++){

}

//TODO
//this function shows the requested calibration image with the detected chessboard corners
void CameraCalibration::showImage(int num)
{


}
