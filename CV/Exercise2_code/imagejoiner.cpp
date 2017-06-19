#include "imagejoiner.h"
#include "multimatcher.h"
#include <limits>

using namespace cv;

ImageJoiner::ImageJoiner(std::vector<cv::Mat> imageList)
{
    //get matches between image pairs and features from multimatcher
    //the multimatcher class calculates feature matches between all image pairs
    //it uses the opencv orb feautre pipline and filters out bad matches using ransac
    //feel free to change the class and plug in your own harris corner detection for example
    //depending on the input images you might have to play around with the ransac settings etc.

    //the filtered matches are returned as a matrix of std::vectors
    //e.g. the matchMatrix[0][1] contains the matches between image 0 and image 1
    //the matchMatrix[2][3] contains the matches between image 2 and image 3 and so on
    //additionally the detected features for every image are returned
    MultiMatcher multimatcher(imageList);
    multimatcher.extractFeatures();
    multimatcher.matchPairs();

    this->matchMatrix = multimatcher.getMatchData();
    this->featureList = multimatcher.getFeatureList();

    //assign the input imagelist as a class member
    this->imageList = imageList;

    //set up a data structure (e.g. std::vector) for the corner points of your images (e.g. upper-left corner etc., not features)
    //Nothing to be done

    //set up a data structure (e.g. std::vector, multi-dim array...) for your transformation matrices
    //Nothing to be done
}

//this function is supposed to find a similarity transformation between image pairs
//find the parameters for each overlapping input image (if in sequence: image0 with image1, image1 with image2 etc)
//save the transformations for the image pairs and your inital image corner points
void ImageJoiner::transformPairwise(){

    //loop over your images and find the similarity transformations (rotation, translation, scaling)
    for(int i=0; i < imageList.size()-1; i++){
        //Solve Ap = b, where 
        //   A = sum_i J^T(x_i) J(x_i) 
        //   b = sum_i J^T(x_i) delta x_i
        //   p = (t_x, t_y, a ,b)
        //A has form 4x4, b has form 4x1, p has form 4x1
        //p = A^-1 b
        Mat A = Mat(4, 4, CV_64F, double(0));
        Mat b = Mat(4, 1, CV_64F, double(0));

        //Matches between ith and i+1th img
        std::vector< cv::DMatch > matches = matchMatrix[i][i+1];
        for(int m = 0; m < matches.size(); m++){
            //Keypoint of first (ith) image
            cv::KeyPoint first = featureList[i][matches[m].queryIdx];
            //Keypoint of second (i+1th) image
            cv::KeyPoint second = featureList[i+1][matches[m].trainIdx];

            //Jacobian of similarity transform of second (i+1th) image
            Mat J = (Mat_<double>(2,4) << 1.0, 0.0, second.pt.x, -second.pt.y, 0.0, 1.0, second.pt.y, second.pt.x);

            //Calculate deltaX
            cv::Point2f deltaX = first.pt - second.pt;
            //Convert to Mat (vector)
            cv::Mat_<double> deltaX_vec(2,1); 
            deltaX_vec(0,0)=deltaX.x; 
            deltaX_vec(1,0)=deltaX.y; 

            //Sum up on A and b
            A += J.t() * J;
            b += J.t() * deltaX_vec;
        }

        //Calculate p
        Mat p = A.inv() * b;

        //Build transformation matrix
        Mat T = Mat(2, 3, CV_64F, double(0));
        T.at<double>(0,0) = 1 + p.at<double>(0, 2); //a
        T.at<double>(1,0) = p.at<double>(0, 3); //b
        T.at<double>(0,1) = -p.at<double>(0, 3); //-b
        T.at<double>(1,1) = 1 + p.at<double>(0, 2); // 1 + a
        T.at<double>(0,2) = p.at<double>(0,0); //t_x
        T.at<double>(1,2) = p.at<double>(0,1); //t_y

        //Save the estimated transformation matrix into transformationMats
        transformationMats.push_back(T);

        //save the inital corner points (Upper-left, upper-right, lower-left, lower right) of your images
        //I just saved it as a cv::Rect
        cv::Rect corners = cv::Rect(0, 0, imageList[i+1].cols, imageList[i+1].rows);
        imageCorners.push_back(corners);
    }
}

//this function will use the estimated tranformations between the image pairs to generate the panography blending
//to simplify the process the first image will be fixed and each other images of your list will be transformed with respect to the first one
//first you have to calculate the transformations in respect to image 0
//e.g. tranformation t01 will transform image1 to be aligned with image0, t12 will transform image2 to be aligned with image1
//to align image2 to image0 you will need to transform image2 with t12 and t01
//the transformed images will have a position in relation to your first image (i.e. they might end up in negative coordinates etc.)
//to find out your final canvas size and position you will have to transform the corner points of your images and use the bounding boxes
cv::Mat ImageJoiner::joinImages(){

    /*********************************************************************************************/
    //data structure to save your final transformation for each image and to save the bounding boxes of transformed corners
    std::vector<Mat> transformList;
    std::vector<cv::Rect> bbList;

    //initialization for image zero with an identity matrix as transformation
    Mat R_prev = (Mat_<double>(2,3) << 1,0,0,0,1,0 );
    transformList.push_back(R_prev);

    //loop over all images
    for(int i=0; i < imageList.size()-1; i++){
        //load your transformation for the current image pair
        //check if the transformation matrix is valid (i.e. not empty)
        Mat T = transformationMats[i];
        if(T.empty()){
            std::cout << "A transformation matrix turned out empty!" << std::endl;
            exit(-1);
        }

        //Take last chained transformation from transformList
        Mat transByNow = transformList.back();
        //Add 0, 0, 1 row at bottom of last chained transformation and raw transformation
        Mat row = (Mat_<double>(1,3) << 0,0,1);
        cv::vconcat(transByNow, row, transByNow);
        cv::vconcat(T, row, T);

        //Multiply both matrices and drop third row afterwards
        Mat completeTransform = (T*transByNow);
        completeTransform = completeTransform.rowRange(0,2);

        
        //transform the corners of the current image with the finial transformation matrix
        //Convert rectangle corners to augmented point vectors
        std::vector<Mat> corners;
        corners.push_back((Mat_<double>(3,1) << imageCorners[i].x, imageCorners[i].y, 1));
        corners.push_back((Mat_<double>(3,1) << imageCorners[i].x+imageCorners[i].width, imageCorners[i].y, 1));
        corners.push_back((Mat_<double>(3,1) << imageCorners[i].x, imageCorners[i].y + imageCorners[i].height, 1));
        corners.push_back((Mat_<double>(3,1) << imageCorners[i].x + imageCorners[i].width, imageCorners[i].y + imageCorners[i].height, 1));

        //Apply transform to corners and save them as points in vector
        std::vector<cv::Point> transformedCorners;
        for(int i=0; i<corners.size(); i++){
            Mat trans = completeTransform * corners[i];
            //std::cout << trans << std::endl;
            transformedCorners.push_back(Point(trans.at<double>(0,0), trans.at<double>(1,0)));
        }

        //Calculate bounding box
        cv::Rect bb = cv::boundingRect(transformedCorners);
        //save the final transformation and bounding box
        transformList.push_back(completeTransform);
        bbList.push_back(bb);
    }

    //sanity check
    //check if there is a complete transformation for every image in your list
    if( transformList.size() != imageList.size() ){
        //std::cout << "Missing trandormation for image pairs!" << std::endl;
        exit(-1);
    }

    /*********************************************************************************************/

    //use the bounding boxes to find the required canvas size
    //additionally you need a "global" translation for all images so they end up on canvas (they need positive coordinates)
    int translationY = 0;
    int translationX = 0;

    //init size of canvas to start image
    int sizeX = imageList[0].rows;
    int sizeY = imageList[0].cols;

    //Determine upper-left and lower-right corner point of every bounding box and append to points std::vector
    std::vector<Point> points;
    for(int i=0; i<bbList.size(); i++){
        points.push_back(bbList[i].tl());
        points.push_back(bbList[i].br());
    }

    //Calculate bounding box of bounding boxes
    cv::Rect bbAll = cv::boundingRect(points);

    //Derive canvas size and global translation
    sizeX = bbAll.width;
    sizeY = bbAll.height;
    if(bbAll.x < 0)
        translationX = -bbAll.x;
    if(bbAll.y < 0)
        translationY = -bbAll.y;

    /*********************************************************************************************/
    //use the transformations, canvas size and global translation to transform and blend the images into a panography

    //create canvas with required size
    Mat blendResult = Mat(Size(sizeX,sizeY), imageList[0].type(), Scalar::all(255));

    //blending factor for linear blending
    float alpha = 0.2;

    //transform and blend
    //loop over all images and warp them
    for(int i=0; i < imageList.size(); i++){
        //get image and according transformation
        Mat img = imageList[i];
        Mat transformation = transformList[i];

        //add "global" translation to transformation matrix
        transformation.at<double>(0,2) += translationX;
        transformation.at<double>(1,2) += translationY;

        //for the blending you will need a binary mask displaying your image loacation on the canvas
        //transform the image into a result image with the size of your canvas
        Mat resultImg;
        warpAffine(img, resultImg, transformation, Size(sizeX,sizeY));

        //create a mask by transforming a completly white image with the size of your original image onto a black result image with the size of the final canvas
        //255 -> white
        Mat maskWhite = Mat(img.size(), CV_8U, 255);
        Mat mask;
        warpAffine(maskWhite, mask, transformation, Size(sizeX,sizeY));

        //to blend your images without blending any white "borders" use masked copys
        //copy your current canvas to a temporary one (blendResultCopy)
        Mat blendResultCopy;
        blendResult.copyTo(blendResultCopy);
        //blend the temporary canvas with your transformed image (linear blending)
        addWeighted(resultImg, 1-alpha, blendResultCopy, alpha, 0.0, blendResultCopy);
        //copy only the pixels belonging to your currently transformed image to the final canvas using the mask you created
        blendResultCopy.copyTo(blendResult, mask);
    }

    //return your panography image
    return blendResult;
}
