#include "cornerdetector.h"
#include <iostream>

using namespace cv;
using namespace std;

/* Default Constructor */
CornerDetector::CornerDetector()
{

}

/* Constructor */
CornerDetector::CornerDetector(cv::Mat &img, int blockSize): img(img)
{
    this->blockSize = blockSize;
}

/* Function to detect corners in given image */
std::vector<cv::KeyPoint> CornerDetector::detectFeatures()
{
    //Compute x and y gradients
    Mat grad_x, grad_y, img_gray;
    cvtColor(img, img_gray, COLOR_BGR2GRAY);
    Sobel(img_gray, grad_x, CV_32F, 1, 0, 3, 1, 0, BORDER_DEFAULT);
    Sobel(img_gray, grad_y, CV_32F, 0, 1, 3, 1, 0, BORDER_DEFAULT);

    //Compute products of derivatives
    Mat prod_xx, prod_yy, prod_xy;
    prod_xx = grad_x.mul(grad_x);
    prod_xy = grad_x.mul(grad_y);
    prod_yy = grad_y.mul(grad_y);

    //Compute the sums of the products of derivatives at each pixel
    //Equivalently, apply unnormalized boxfilter
    Mat prod_xx_sum, prod_xy_sum, prod_yy_sum;
    boxFilter(prod_xx, prod_xx_sum, CV_32F, Size(blockSize, blockSize), Point(-1,-1), false);
    boxFilter(prod_xy, prod_xy_sum, CV_32F, Size(blockSize, blockSize), Point(-1,-1), false);
    boxFilter(prod_yy, prod_yy_sum, CV_32F, Size(blockSize, blockSize), Point(-1,-1), false);

    //Compute R = Det(H) - k(Trace(H))^2
    Mat R = Mat::zeros(img.size(), CV_32F);
    Mat H = Mat::zeros(2, 2, CV_32F);
    float k = 0.05;
    for(int i = 0; i<img.rows; i++){
        for(int j = 0; j<img.cols; j++){
            // ad - bc
            float det = prod_xx_sum.at<float>(i,j) * prod_yy_sum.at<float>(i,j) - prod_xy_sum.at<float>(i,j) * prod_xy_sum.at<float>(i,j);
            // a + d
            float trace = prod_xx_sum.at<float>(i,j) + prod_yy_sum.at<float>(i,j);
            R.at<float>(i,j) = det - k * trace*trace;
        }
    } 
    namedWindow("Image", WINDOW_AUTOSIZE);
    imshow("Image", R);
    waitKey(0);
    
    //Threshold
    float thresh = 10000.0;
    for(int i = 0; i<img.rows; i++){
        for(int j = 0; j<img.cols; j++){
            if (R.at<float>(i,j) > thresh)
            {
                //Non-maximum suppression
                int max_x, max_y;
                float max = 0.0;
                for(int x = -3; x<4; x++){
                    for(int y = -3; y<4; y++){
                        if(i+x >= 0 && j+y >=0 && i+x < img.rows && j+y <img.cols){
                            if (R.at<float>(i+x,j+y) > max){
                                max_x = x;
                                max_y = y;
                                max = R.at<float>(i+x, j+y);
                            } 
                        } 
                    }
                }

                if (max_x == 0 && max_y == 0){
                    KeyPoint k(i,j,1);
                    keyPoints.push_back(k);
                }
            }
        }
    }
    return this->keyPoints;
}
