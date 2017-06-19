#include "writeoff.h"
#include <iostream>
#include <fstream>

WriteOff::WriteOff(){

    this->R = Mat::eye(3, 3, CV_64F);
    this->t = Mat::eye(3, 1, CV_64F);

    //generate model
    generateCamPoints(camModel);

}

WriteOff::WriteOff(const Mat &R, const Mat &t, const string &filename, float ratioXY, float scale){

    this->R = R.clone();
    this->t = t.clone();
    this->ratioXY = ratioXY;
    this->scale = scale;

    generateCamPoints(camModel, ratioXY, scale);
    transform(R,t);
    writeFile(filename);

}

void WriteOff::transform(const Mat &R, const Mat &t)
{

    //translate
    float x,y,z;
    for(int i = 0; i<camModel.size(); i++){
        x = camModel[i].x-t.at<double>(0,0);
        y = camModel[i].y-t.at<double>(0,1);
        z = camModel[i].z-t.at<double>(0,2);

        transformedModel.push_back(Point3f(x,y,z));
    }

    buildTransformationMatrix(R);
    perspectiveTransform(transformedModel, transformedModel, P);
}

void WriteOff::writeFile(const string &filename)
{
    writeFile(filename, camModel, transformedModel);
}


void WriteOff::writeFile(const string &filename, const std::vector<Point3f> cam1, const std::vector<Point3f> cam2)
{
    //generate off file
    int numPoints = cam1.size()+cam2.size();
    ofstream offFile (filename.c_str());
    if (offFile.is_open()){

        offFile << "COFF\n";
        offFile << numPoints <<" 0 0\n";

        for(int i = 0; i<cam1.size(); i++){
            offFile << cam1[i].x << " "<< cam1[i].y << " "<< cam1[i].z << " " << 255 << " "<< 255 << " "<< 255 << "\n";
        }

        for(int i = 0; i<cam2.size(); i++){
            offFile << cam2[i].x << " "<< cam2[i].y << " "<< cam2[i].z << " " << 255 << " "<< 255 << " "<< 0 << "\n";
        }
        offFile.close();
    }
    else{
        cout << "Unable to open file" << endl;
    }

    cout << "OFF file written!" << endl;

}

void WriteOff::buildTransformationMatrix(const Mat &R)
{
    //Add trandformation to matrix
    this->P = Mat::eye(4, 4, CV_64F);
    for (int i = 0; i < 3; i++){
        for (int j = 0; j < 3; j++){
            this->P.at<double>(i,j) = R.at<double>(i,j);
        }
    }

}


void WriteOff::lineOfPoints(const Point3f start, const Point3f end, int numPoints, std::vector<Point3f> &pointVector){

    Point3f distance = end-start;
    Point3f step = Point3f(distance.x/float(numPoints),distance.y/float(numPoints),distance.z/float(numPoints)) ;

    Point3f newPoint = start;
    for(int i = 0; i<numPoints; i++){
        newPoint += step;
        pointVector.push_back(newPoint);
    }

}

void WriteOff::generateCamPoints(std::vector<Point3f> &pointVector, float ratioXY, float scale){

    int linePoints = int(150 * scale);
    float camSize = -0.2*scale;

    //view direction
    Point3f start = Point3f(0.0,0.0,0.0);
    Point3f end = Point3f(0.0,0.0,-2.5)*scale;
    lineOfPoints(start, end, int(linePoints*2.5), pointVector);

    //image plane
    float planeSizeX = 0.2*scale*ratioXY;
    float planeSizeY = 0.2*scale;
    start = Point3f(planeSizeX,planeSizeY,camSize);
    end= Point3f(planeSizeX,-planeSizeY,camSize);
    lineOfPoints(start, end, linePoints, pointVector);
    start= Point3f(-planeSizeX,planeSizeY,camSize);
    end= Point3f(-planeSizeX,-planeSizeY,camSize);
    lineOfPoints(start, end, linePoints, pointVector);
    start= Point3f(-planeSizeX,planeSizeY,camSize);
    end= Point3f(planeSizeX,planeSizeY,camSize);
    lineOfPoints(start, end, linePoints, pointVector);
    start= Point3f(-planeSizeX,-planeSizeY,camSize);
    end= Point3f(planeSizeX,-planeSizeY,camSize);
    lineOfPoints(start, end, linePoints, pointVector);

    //origin to plane
    start = Point3f(0.0,0.0,0.0);
    end= Point3f(planeSizeX,planeSizeY,camSize);
    lineOfPoints(start, end, linePoints, pointVector);
    end= Point3f(-planeSizeX,-planeSizeY,camSize);
    lineOfPoints(start, end, linePoints, pointVector);
    end= Point3f(-planeSizeX,planeSizeY,camSize);
    lineOfPoints(start, end, linePoints, pointVector);
    end= Point3f(planeSizeX,-planeSizeY,camSize);
    lineOfPoints(start, end, linePoints, pointVector);

}

