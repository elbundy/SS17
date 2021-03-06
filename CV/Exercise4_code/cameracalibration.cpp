#include "cameracalibration.h"

//constructor setting given parameters and default values
CameraCalibration::CameraCalibration(const vector<Mat> imageList, Size boardSize, float squareSize): imageList{imageList}, boardSize{boardSize}, squareSize{squareSize}
{
    this->cameraMatrix = Mat::eye(3, 3, CV_64F);
    this->distCoeffs = Mat::zeros(8, 1, CV_64F);
}

//for reuse set new imageList, clear previous data
void CameraCalibration::setImageList(const vector<Mat> imageList)
{
    this->imageList = imageList;
    this->imagePoints = vector<vector<Point2f>>();
    this->objectPoints = vector<vector<Point3f>>();
}

//this function will return cameraMatrix and distortion coefficients
void CameraCalibration::calibrate(Mat &cameraMatrix, Mat &distCoeffs)
{

    //implement functions finding the chessboard corners in the images and generating the object corners according to board parameters
    findImagePoints();
    generateObjectPoints();

    //Find intrinsic and extrinsic camera parameters - calibrateCamera
    double reprojectionError = cv::calibrateCamera(this->objectPoints, this->imagePoints, imageList[0].size(), this->cameraMatrix, this->distCoeffs, this->rvecs, this->tvecs);

    //print returned re-projection error
    cout << "Re-projection error: " << reprojectionError<< endl;

    cameraMatrix = this->cameraMatrix.clone();
    distCoeffs = this->distCoeffs.clone();
}

//find the chessboard corners in the calibration images
//draw the detected corners onto the images
//the final image points should be pushed into this->imagePoints
void CameraCalibration::findImagePoints()
{
    int num = 0;
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

            //Append found points to member vector
            this->imagePoints.push_back(imagePoints);

            //Show
            showImage(num);
        }
        num += 1;
    }
    

}

//generate the object points of the calibration pattern
//use boardSize and squareSize
//they should be saved into this->objectPoints
void CameraCalibration::generateObjectPoints()
{
    vector<Point3f> objPoints;

    //Worlds origin is top left corner point
    for(std::vector<Mat>::iterator it = imageList.begin(); it < imageList.end(); it++){
        objPoints.clear();
        for(int row = 0; row < this->boardSize.height; row++){
            for(int col = 0; col < this->boardSize.width; col++){
                Point3f point(row*this->squareSize, col*this->squareSize, 0); 
                objPoints.push_back(point);
                //std::cout << point << std::endl;
            }
        }
        this->objectPoints.push_back(objPoints);
        //std::cout << "----" << std::endl;
    }
}

//this function shows the requested calibration image with the detected chessboard corners
void CameraCalibration::showImage(int num)
{
    Mat im = this->imageList[num].clone();
    drawChessboardCorners(im, this->boardSize, this->imagePoints[num], true);
    imshow("Detected corners", im);
    waitKey(0);
    cv::destroyWindow("Detected corners");
}
