// system includes
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <algorithm>

// library includes
#include <opencv2/opencv.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"

#include "writeoff.h"

using namespace std;
using namespace cv;

//estimate the rotation and translation of the camera given the essential matrix E
//see:
//Richard Hartley and Andrew Zisserman (2003). Multiple View Geometry in computer vision
//http://isit.u-clermont1.fr/~ab/Classes/DIKU-3DCV2/Handouts/Lecture16.pdf
void extractCameraFromE(const Mat& E, Mat& R, Mat& t, int solution = 0){
    //extract camera rotation and translation from essential matrix

    //1.) Compute SVD
    Mat_<double> w, U, Vt;
    SVD::compute(E, w, U, Vt);

    //2.) Set up W and Z
    Mat_<double> W = Mat_<double>::zeros(3, 3);
    W(0,1) = -1.0;
    W(1,0) = 1.0;

    Mat_<double> Z = Mat_<double>::zeros(3, 3);
    Z(0,1) = 1.0;
    Z(1,0) = -1.0;

    //there are 4 possible solutions for R, t
    //see e.g. hartley/zisserman
    if(solution == 0){
        R = U * W * Vt;
        t = U.col(2);
    }else if(solution == 1){
        R = U * W * Vt;
        t = -1.0 * U.col(2);
    }else if(solution == 2){
        R = U * W.t() * Vt;
        t = U.col(2);
    }else{
        R = U * W.t() * Vt;
        t = -1.0 * U.col(2);
    }
}

//calculate z component of single point
//transform 3D point to check if it ends up infront of both cameras
bool checkDepth(Point2f point1, Point2f point2, const Mat& R, const Mat& t){
    //calculate z component of point1
    std::vector<cv::Point2f> cam0pnts;
    cam0pnts.push_back(point1);
    std::vector<cv::Point2f> cam1pnts;
    cam1pnts.push_back(point2);

    cv::Mat_<double> projectionMat1 = cv::Mat_<double>::zeros(3,4);
    projectionMat1(0,0) = 1.0;
    projectionMat1(1,1) = 1.0;
    projectionMat1(2,2) = 1.0;
    //std::cout << projectionMat1 << std::endl;

    cv::Mat_<double> projectionMat2;
    //std::cout << R.size().height << t.size().height << std::endl;
    cv::hconcat(R, t, projectionMat2);
    //std::cout << projectionMat2 << std::endl;
    
    cv::Mat pnts3D(4, 1, CV_64F);
    cv::triangulatePoints(projectionMat1, projectionMat2, cam0pnts, cam1pnts, pnts3D);
    pnts3D.convertTo(pnts3D, CV_64F); //????? bug

    //transform 3D point / depth
    Mat_<double> proj1 = projectionMat1 * pnts3D;
    Mat_<double> proj2 = projectionMat2 * pnts3D;

    //return true if point infront of both cameras -> both z components positive
    if(proj1(2,0) >= 0.0 && proj2(2,0) >= 0.0){
        return true;
    }
    return false;
}


//checks if rotation is proper via determiant
bool checkProperRotation(const Mat &R){

    //indicates reflection
    if(determinant(R) < 0){
        cout << "Improper Rotation!" << endl;
        return false;
    }
    return true;
}

int main(int argc, char** argv )
{

    //load two images
    //the second of the provided images is translated in x direction and rotated around y
    Mat img1 = imread("./img.png");
    Mat img2 = imread("./img_tranX_rotY.png");

    /***********************Feature Matching**************************/
    std::vector<cv::DMatch> matches;
    std::vector<cv::KeyPoint> features1;
    std::vector<cv::KeyPoint> features2;

    Mat descriptors1;
    Mat descriptors2;

    //Detector and descriptor
    Ptr<FeatureDetector> detector = FeatureDetector::create("GFTT"); //GFTT PyramidGFTT ORB Dense SimpleBlob STAR MSER HARRIS GridFAST FAST PyramidSTAR
    Ptr<DescriptorExtractor> extractor = DescriptorExtractor::create("ORB"); //ORB FREAK BRISK BRIEF

    detector->detect( img1, features1, Mat() );
    detector->detect( img2, features2, Mat() );

    extractor->compute( img1, features1, descriptors1 );
    extractor->compute( img2, features2, descriptors2 );

    BFMatcher matcher(cv::NORM_HAMMING, false);
    matcher.match( descriptors1, descriptors2, matches );

    //pre filter matches via distance
    Mat distMat = Mat::zeros(matches.size(),1,CV_32F);
    for( int i = 0; i < matches.size(); i++ ){
        distMat.at<float>(i,0) = matches[i].distance;
    }

    Scalar stddev, meanMat;
    meanStdDev(distMat, meanMat, stddev);
    float mean = meanMat[0];

    //filtered_matches
    std::vector< DMatch > filtered_matches;
    for( int i = 0; i < matches.size(); i++ ){
        if( matches[i].distance < (mean) ){
            filtered_matches.push_back( matches[i]);
        }
    }

    /*******************************************************************/

    /**************estimate fundamental matrix**************************/

    //ransac options
    double distance = 0.5;
    double confidence = 0.99;

    //Convert keypoints matched keypoints into Point2f
    std::vector<cv::Point2f> points1, points2;
    for (std::vector<cv::DMatch>::const_iterator it= filtered_matches.begin();
         it!= filtered_matches.end(); ++it) {

        // Get the position of left keypoints
        float x= features1[it->queryIdx].pt.x;
        float y= features1[it->queryIdx].pt.y;
        points1.push_back(cv::Point2f(x,y));
        // Get the position of right keypoints
        x= features2[it->trainIdx].pt.x;
        y= features2[it->trainIdx].pt.y;
        points2.push_back(cv::Point2f(x,y));
    }

    //estimate fundamental matrix using cv::findFundamentalMat()
    //use CV_FM_RANSAC
    //estimation is very sensitive to number of matched features, miss-matched features etc. -> dont expect too much

    std::vector<uchar> inliers(points1.size(),0);
    std::fill(inliers.begin(), inliers.end(), 1);
    Mat F = cv::findFundamentalMat(points1, points2, FM_RANSAC, distance, confidence);

    /*******************************************************************/

    /**************visualize inliers************************************/

    //get good matches from inliers vector
    //draw good matches
    std::vector<cv::DMatch> good_matches;
    std::vector<uchar>::const_iterator itIn= inliers.begin();
    std::vector<cv::DMatch>::const_iterator itM= filtered_matches.begin();
    // for all matches
    for ( ;itIn!= inliers.end(); ++itIn, ++itM) {

        if (*itIn) { // it is a valid match

            good_matches.push_back(*itM);
        }
    }

    //draw filtered matches (ransac inliers)
    Mat img_matches;
    drawMatches( img1, features1, img2, features2, good_matches, img_matches );
    resize(img_matches, img_matches,Size(),0.5,0.5);
    imshow("Matches", img_matches );
    //waitKey(0);

    /*******************************************************************/

    /**************set intrinsic camera matrix**************************/

    //set the intrinsic camera matrix from the given parameters
    //inform yourself about the intrinsic camera matrix
    //set it
    //the camera parameters for the provided images are:
    //focal length = 35 mm
    //sensor width = 32 mm
    //Resolution 2048x1536
    //no skew etc.
    //    
    //    | f_x s  x_0 |
    // K =| 0  f_y y_0 |
    //    | 0   0   1  |
    // f: focal length, x/y: principal point offset, s: skew
    //
    // from de.mathworks:
    // f_x = F (in mm) / pixel size
    // from answers.opencv
    //// focal_length_in_pixel = (focal_length_mm / sensor_width_mm)*img_width in pixels
    double focal = 35.0;
    double sensor = 32.0;
    double res_width = 2048.0;
    double res_height = 1536.0;

    double f_x, f_y;     
    f_x = (focal/sensor);
    f_y = (focal/sensor);

    double x_0, y_0;
    x_0 = res_width / 2;
    y_0 = res_height / 2;

    //double s = 0.0;
    Mat_<double> K = Mat_<double>::zeros(3, 3);
    K(0,0) = f_x;
    //K(0,1) = s;
    K(0,2) = x_0;
    K(1,1) = f_y;
    K(1,2) = y_0;
    K(2,2) = 1;
    //std::cout << K << std::endl;

    /*******************************************************************/

    /************** get essential matrix********************************/

    //get essential matrix from fundamental matrix and camera instrinsic matrix
    //E = K'^T F K (here: K' = K), K is the intrinsic camera matrix

    Mat E = K.t() * F * K;

    /*******************************************************************/

    /****************** get inliers out of good matches ****************/
    //-- Localize the object
    std::vector<Point2f> point1g;
    std::vector<Point2f> point2g;

    for( int k = 0; k < good_matches.size(); k++ )
    {
        //-- Get the keypoints from the good matches
        point1g.push_back( features1[ good_matches[k].queryIdx ].pt );
        point2g.push_back( features2[ good_matches[k].trainIdx ].pt );
    }


    //Select corresponding point in img1 and img2.
    //Normalize points (to image resolution) (i.e. between 0.0 and 1.0)
    //multiple points could be used to be more robust
    Point2f point1 = Point2f(point1g[1].x / img1.rows, point1g[1].y / img1.cols);
    Point2f point2 = Point2f(point2g[1].x / img2.rows, point2g[1].y / img2.cols);

    /*******************************************************************/

    /******************* R,t from essential matrix *********************/

    //estimate rotation and translation from the essential matrix
    //reconstruct the depth of a single corresponding point to find valid solution
    //check all solutions for the right one
    Mat R = Mat::eye(3, 3, CV_64F);
    Mat t = Mat::zeros(3, 1, CV_64F);

    int solution = 0;

    while(true){
        //extract the camera roatation and translation from the essential matrix
        extractCameraFromE(E,R,t,solution);
        //check for reflective rotation
        if (!checkProperRotation(R)){
            //TODO fix the case of improper rotation (i.e. det(R) = -1)
        }

        //reconstruct z component for chosen and normalized points (point1, point2) and check if it is positive for both cameras
        //if infront of both camreas you found the right of the 4 possible solutions
        if(checkDepth(point1, point2, R, t))
            break;

        solution += 1;
        if(solution == 4){
            cout << "No Solution found!" << endl;
            return false;
        }
    }

    //this class writes an off file visualizing the two cameras
    //the first camera is set to (0,0,0) and a second camera translated and rotated realtive to the first with R and t
    //check your results by vieweing the off file in meshlab
    //a valid result is saved in tranX_rotZ
    //don't worry if your results arent exactly the same, coordinate systems might not match
    //you might need to adjust your translation (multiply -1.0)
    WriteOff OFF(R, t, "cameras.off", (4.0/3.0), 1.0);

    waitKey(0);
    return 0;
}
