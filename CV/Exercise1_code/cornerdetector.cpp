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
    //Compute x and y gradients via Sobel filter convolution
    Mat grad_x, grad_y, img_gray;
    cvtColor(img, img_gray, COLOR_BGR2GRAY);
    Sobel(img_gray, grad_x, CV_32F, 1, 0, 3, 1, 0, BORDER_DEFAULT);
    Sobel(img_gray, grad_y, CV_32F, 0, 1, 3, 1, 0, BORDER_DEFAULT);

    //namedWindow("Image", WINDOW_AUTOSIZE);
    //imshow("Grad X", grad_x);
    //imshow("Grad Y", grad_y);  
    //waitKey(0);

    //Compute products of derivatives
    Mat prod_xx, prod_yy, prod_xy;
    prod_xx = grad_x.mul(grad_x);
    prod_xy = grad_x.mul(grad_y);
    prod_yy = grad_y.mul(grad_y);

    //Compute the sums of the products of derivatives at each pixel
    //In this case, Gaussian filter was used (as mentioned in Szeliski)
    Mat prod_xx_sum, prod_xy_sum, prod_yy_sum;
    GaussianBlur(prod_xx, prod_xx_sum, Size(blockSize, blockSize), 0, 0);
    GaussianBlur(prod_xy, prod_xy_sum, Size(blockSize, blockSize), 0, 0);
    GaussianBlur(prod_yy, prod_yy_sum, Size(blockSize, blockSize), 0, 0);

    //Compute R = Det(H) - k(Trace(H))^2
    Mat R = Mat::zeros(img.size(), CV_32F);
    Mat H = Mat::zeros(2, 2, CV_32F);
    float k = 0.1;
    //at(row, col) indexing
    //in opencv, row is y coordinate, col is x coordinate
    for(int x = 0; x<img.cols; x++){
        for(int y = 0; y<img.rows; y++){
            // ad - bc
            float det = prod_xx_sum.at<float>(y,x) * prod_yy_sum.at<float>(y,x) - prod_xy_sum.at<float>(y,x) * prod_xy_sum.at<float>(y,x);
            // a + d
            float trace = prod_xx_sum.at<float>(y,x) + prod_yy_sum.at<float>(y,x);
            R.at<float>(y,x) = det - k * trace*trace;
        }
    } 
   
    //Threshold
    float thresh = 10000.0;
    for(int x = 0; x<img.cols; x++){
        for(int y = 0; y<img.rows; y++){
            if (R.at<float>(y,x) > thresh)
            {
                //Non-maximum suppression
                int max_x, max_y;
                float max = 0.0;
                for(int x_off = -5; x_off<6; x_off++){
                    for(int y_off = -5; y_off<6; y_off++){
                        if(x+x_off >= 0 && y+y_off >=0 && x+x_off < img.cols && y+y_off < img.rows){
                            if (R.at<float>(y+y_off,x+x_off) > max){
                                max_x = x_off;
                                max_y = y_off;
                                max = R.at<float>(y+y_off, x+x_off);
                            } 
                        } 
                    }
                }

                if (max_x == 0 && max_y == 0){
                    KeyPoint k(x,y,1);
                    keyPoints.push_back(k);
                }
            }
        }
    }
    return this->keyPoints;
}
