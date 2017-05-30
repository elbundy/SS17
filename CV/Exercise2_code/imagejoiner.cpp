#include "imagejoiner.h"
#include "multimatcher.h"

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

    //TODO
    //set up a data structure (e.g. std::vector) for the corner points of your images (e.g. upper-left corner etc., not features)

    //TODO
    //set up a data structure (e.g. std::vector, multi-dim array...) for your transformation matrices
}

//TODO
//this function is supposed to find a similarity transformation between image pairs
//find the parameters for each overlapping input image (if in sequence: image0 with image1, image1 with image2 etc)
//save the transformations for the image pairs and your inital image corner points
void ImageJoiner::transformPairwise(){

    //loop over your images and find the similarity transformations (rotation, translation, scaling)
    for(int i=0; i < imageList.size()-1; i++){
            //use the matches from the matchMatrix
            //estimate the ridgid transformation (only rotation, scaling and translation for the panography application)

            /*Mat matchImg;
            cv::drawMatches(imageList[i], featureList[i], imageList[i+1], featureList[i+1], matchMatrix[i][i+1], matchImg);
            namedWindow("Image", CV_WINDOW_NORMAL);
            imshow("Image", matchImg);
            waitKey(0);
            break;
            */

            //Solve Ap = b, where 
            //   A = sum_i J^T(x_i) J(x_i) 
            //   b = sum_i J^T(x_i) delta x_i
            //   p = (t_x, t_y, a ,b)
            //A has form 4x4, b has form 4x1, p has form 4x1
            //p = A^-1 b
            Mat A = Mat(4, 4, CV_32F, float(0));
            Mat b = Mat(4, 1, CV_32F, float(0));

            //Matches between i and i+1 img
            std::vector< cv::DMatch > matches = matchMatrix[i][i+1];
            for(int m = 0; m < matches.size(); m++){
                cv::KeyPoint first = featureList[i][matches[m].queryIdx];
                cv::KeyPoint second = featureList[i+1][matches[m].trainIdx];
                //Jacobian of similarity transform of second image
                Mat J = (Mat_<float>(2,4) << 1.0, 0.0, second.pt.x, -second.pt.y, 0.0, 1.0, second.pt.y, second.pt.x);
                //Calculate deltaX vector
                cv::Point2f deltaX = first.pt - second.pt;
                cv::Mat_<float> deltaX_vec(2,1); 
                deltaX_vec(0,0)=deltaX.x; 
                deltaX_vec(1,0)=deltaX.y; 

                A += J.t() * J;
                b += J.t() * deltaX_vec;
            }
            Mat p = A.inv() * b;
            std::cout << p; 

            //Build transformation matrix
            Mat T = Mat(2, 3, CV_32F, float(0));
            T(0,0) = 1 + p(0, 2); //a
            T(1,0) = p(0, 3); //b
            T(0,1) = -p(0, 3);
            T(1,1) = 1 + p(0, 2);
            T(0,2) = p(0,0); //t_x
            T(1,2) = p(0,1); //t_y

            //save the estimated transformation matrices into your data-structure
            transformationMats.push_back(T);
        //TODO
        //save the inital corner points (Upper-left, upper-right, lower-left, lower right) of your images


    }
}

//TODO
//this function will use the estimated tranformations between the image pairs to generate the panography blending
//to simplify the process the first image will be fixed and each other images of your list will be transformed with respect to the first one
//first you have to calculate the transformations in respect to image 0
//e.g. tranformation t01 will transform image1 to be aligned with image0, t12 will transform image2 to be aligned with image1
//to align image2 to image0 you will need to transform image2 with t12 and t01
//the transformed images will have a position in relation to your first image (i.e. they might end up in negative coordinates etc.)
//to find out your final canvas size and position you will have to transform the corner points of your images and use the bounding boxes
cv::Mat ImageJoiner::joinImages(){

    /*********************************************************************************************/
    //TODO

    //data structure to save your final transformation for each image and to save the bounding boxes of transformed corners
    std::vector<Mat> transformList;
    std::vector<cv::Rect>  bbList;

    //TODO
    //initialization for image zero with an identity matrix as transformation
    Mat R_prev = (Mat_<double>(2,3) << 1,0,0,0,1,0 );
    transformList.push_back(R_prev);

    //loop over all images
    for(int i=0; i < imageList.size()-1; i++){


        //TODO
        //load your transformation for the current image pair
        //check if the transformation matrix is valid (i.e. not empty)


        //TODO
        //multiply the transformations for the current image as needed to get the complete transformation in respect to image0
        Mat completeTransform;


        //TODO
        //transform the corners of the current image with the finial transformation matrix
        //calculate the bounding box of the resulting points
        cv::Rect bb;


        //TODO
        //sae the final transformation and bounding box
        transformList.push_back(completeTransform);
        bbList.push_back(bb);

    }


    //sanity check
    //check if there is a complete transformation for every image in your list
    if( transformList.size() != imageList.size() ){
        std::cout << "Missing trandormation for image pairs!" << std::endl;
        exit(-1);
    }

    /*********************************************************************************************/
    //TODO

    //use the bounding boxes to find the required canvas size
    //additionally you need a "global" translation for all images so they end up on canvas (they need positive coordinates)
    int translationY = 0;
    int translationX = 0;

    //init size of canvas to start image
    int sizeX = imageList[0].rows;
    int sizeY = imageList[0].cols;

    //TODO
    //check all bounding boxes to get the final canvas size required and "global" translation so all images fit the canvas



    /*********************************************************************************************/
    //TODO
    //use the transformations, canvas size and global translation to transform and blend the images into a panography

    //create canvas with required size
    Mat blendResult = Mat(Size(sizeX,sizeY), imageList[0].type(), Scalar::all(255));

    //blending factor for linear blending
    float alpha = 0.2;

    //TODO transform and blend
    //loop over all images and warp them

    //TODO
    //get image and according transformation

    //TODO
    //add "global" translation to transformation matrix


    //TODO
    //for the blending you will need a binary mask displaying your image loacation on the canvas
    //transform the image into a result image with the size of your canvas


    //TODO
    //create a mask by transforming a completly white image with the size of your original image onto a black result image with the size of the final canvas


    //TODO
    //to blend your images without blending any white "borders" use masked copys
    //copy your current canvas to a temporary one
    //blend the temporary canvas with your transformed image (linear blending)
    //copy only the pixels belonging to your currently transformed image to the final canvas using the mask you created


    //return your panography image
    return blendResult;

}
