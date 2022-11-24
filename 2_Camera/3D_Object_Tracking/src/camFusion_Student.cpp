
#include <iostream>
#include <algorithm>
#include <numeric>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "camFusion.hpp"
#include "dataStructures.h"

using namespace std;


// Create groups of Lidar points whose projection into the camera falls into the same bounding box
void clusterLidarWithROI(std::vector<BoundingBox> &boundingBoxes, std::vector<LidarPoint> &lidarPoints, float shrinkFactor, cv::Mat &P_rect_xx, cv::Mat &R_rect_xx, cv::Mat &RT)
{
    // loop over all Lidar points and associate them to a 2D bounding box
    cv::Mat X(4, 1, cv::DataType<double>::type);
    cv::Mat Y(3, 1, cv::DataType<double>::type);

    for (auto it1 = lidarPoints.begin(); it1 != lidarPoints.end(); ++it1)
    {
        // assemble vector for matrix-vector-multiplication
        X.at<double>(0, 0) = it1->x;
        X.at<double>(1, 0) = it1->y;
        X.at<double>(2, 0) = it1->z;
        X.at<double>(3, 0) = 1;

        // project Lidar point into camera
        Y = P_rect_xx * R_rect_xx * RT * X;
        cv::Point pt;
        // pixel coordinates
        pt.x = Y.at<double>(0, 0) / Y.at<double>(2, 0); 
        pt.y = Y.at<double>(1, 0) / Y.at<double>(2, 0); 

        vector<vector<BoundingBox>::iterator> enclosingBoxes; // pointers to all bounding boxes which enclose the current Lidar point
        for (vector<BoundingBox>::iterator it2 = boundingBoxes.begin(); it2 != boundingBoxes.end(); ++it2)
        {
            // shrink current bounding box slightly to avoid having too many outlier points around the edges
            cv::Rect smallerBox;
            smallerBox.x = (*it2).roi.x + shrinkFactor * (*it2).roi.width / 2.0;
            smallerBox.y = (*it2).roi.y + shrinkFactor * (*it2).roi.height / 2.0;
            smallerBox.width = (*it2).roi.width * (1 - shrinkFactor);
            smallerBox.height = (*it2).roi.height * (1 - shrinkFactor);

            // check wether point is within current bounding box
            if (smallerBox.contains(pt))
            {
                enclosingBoxes.push_back(it2);
            }

        } // eof loop over all bounding boxes

        // check wether point has been enclosed by one or by multiple boxes
        if (enclosingBoxes.size() == 1)
        { 
            // add Lidar point to bounding box
            enclosingBoxes[0]->lidarPoints.push_back(*it1);
        }

    } // eof loop over all Lidar points
}

/* 
* The show3DObjects() function below can handle different output image sizes, but the text output has been manually tuned to fit the 2000x2000 size. 
* However, you can make this function work for other sizes too.
* For instance, to use a 1000x1000 size, adjusting the text positions by dividing them by 2.
*/
void show3DObjects(std::vector<BoundingBox> &boundingBoxes, cv::Size worldSize, cv::Size imageSize, bool bWait)
{
    // create topview image
    cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(255, 255, 255));

    for(auto it1=boundingBoxes.begin(); it1!=boundingBoxes.end(); ++it1)
    {
        // create randomized color for current 3D object
        cv::RNG rng(it1->boxID);
        cv::Scalar currColor = cv::Scalar(rng.uniform(0,150), rng.uniform(0, 150), rng.uniform(0, 150));

        // plot Lidar points into top view image
        int top=1e8, left=1e8, bottom=0.0, right=0.0; 
        float xwmin=1e8, ywmin=1e8, ywmax=-1e8;
        for (auto it2 = it1->lidarPoints.begin(); it2 != it1->lidarPoints.end(); ++it2)
        {
            // world coordinates
            float xw = (*it2).x; // world position in m with x facing forward from sensor
            float yw = (*it2).y; // world position in m with y facing left from sensor
            xwmin = xwmin<xw ? xwmin : xw;
            ywmin = ywmin<yw ? ywmin : yw;
            ywmax = ywmax>yw ? ywmax : yw;

            // top-view coordinates
            int y = (-xw * imageSize.height / worldSize.height) + imageSize.height;
            int x = (-yw * imageSize.width / worldSize.width) + imageSize.width / 2;

            // find enclosing rectangle
            top = top<y ? top : y;
            left = left<x ? left : x;
            bottom = bottom>y ? bottom : y;
            right = right>x ? right : x;

            // draw individual point
            cv::circle(topviewImg, cv::Point(x, y), 4, currColor, -1);
        }

        // draw enclosing rectangle
        cv::rectangle(topviewImg, cv::Point(left, top), cv::Point(right, bottom),cv::Scalar(0,0,0), 2);

        // augment object with some key data
        char str1[200], str2[200];
        sprintf(str1, "id=%d, #pts=%d", it1->boxID, (int)it1->lidarPoints.size());
        putText(topviewImg, str1, cv::Point2f(left-50, bottom+150), cv::FONT_ITALIC, 1, currColor);
        sprintf(str2, "xmin=%2.2f m, yw=%2.2f m", xwmin, ywmax-ywmin);
        putText(topviewImg, str2, cv::Point2f(left-50, bottom+200), cv::FONT_ITALIC, 1, currColor);  
    }

    // plot distance markers
    float lineSpacing = 2.0; // gap between distance markers
    int nMarkers = floor(worldSize.height / lineSpacing);
    for (size_t i = 0; i < nMarkers; ++i)
    {
        int y = (-(i * lineSpacing) * imageSize.height / worldSize.height) + imageSize.height;
        cv::line(topviewImg, cv::Point(0, y), cv::Point(imageSize.width, y), cv::Scalar(255, 0, 0));
    }

    // display image
    string windowName = "3D Objects";
    cv::namedWindow(windowName, 1);
    cv::imshow(windowName, topviewImg);

    if(bWait)
    {
        cv::waitKey(0); // wait for key to be pressed
    }
}


// associate a given bounding box with the keypoints it contains
void clusterKptMatchesWithROI(BoundingBox &boundingBox, std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, std::vector<cv::DMatch> &kptMatches)
{
    vector<cv::DMatch> kptMatchesUnfilterd;   
    vector<double> distanceMatches;
    for (auto it = kptMatches.begin(); it != kptMatches.end(); ++it){
        cv::KeyPoint kptCurr = kptsCurr.at(it->trainIdx);
        cv::KeyPoint kptPrev = kptsPrev.at(it->queryIdx);
        if (boundingBox.roi.contains(kptCurr.pt)){
            kptMatchesUnfilterd.push_back(*it);
            distanceMatches.push_back(norm(kptCurr.pt - kptPrev.pt));
        }
    }

    // filter out outliers that are far away from the mean
    double filterRatio = 1.0;
    double distanceMean = accumulate(distanceMatches.begin(), distanceMatches.end(), 0.0)/distanceMatches.size();
    for(int i = 0; i<distanceMatches.size(); ++i){
        if (distanceMatches[i] < distanceMean * filterRatio){
            boundingBox.keypoints.push_back(kptsCurr[kptMatches[i].trainIdx]);
            boundingBox.kptMatches.push_back(kptMatchesUnfilterd[i]);
        }
    }
    bool bDebug = true;
    if (bDebug)
    {
        std::cout << "Input kptMatches.size=" << kptMatches.size() << std::endl;
        std::cout << "Output boundingBox.kptMatches.size=" << boundingBox.kptMatches.size() << std::endl;
    }        
}


// Compute time-to-collision (TTC) based on keypoint correspondences in successive images
void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, 
                      std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC, cv::Mat *visImg)
{
    vector<double> distRatios;  // stores the distance ratios for all keypoints between curr. and prev. frame
    for (auto it1 = kptMatches.begin(); it1 != kptMatches.end()-1; ++it1){
        // outer loop

        // get current keypoint and its matched partner in the prev. frame
        cv::KeyPoint kptOuterCurr = kptsCurr.at(it1->trainIdx);
        cv::KeyPoint kptOuterPrev = kptsPrev.at(it1->queryIdx);
        
        for (auto it2 = kptMatches.begin()+1; it2 != kptMatches.end(); ++it2){
            // inner loop
            double minDist = 100.0; // min required distance

            // get next keypoint and its matched partner in the prev. frame
            cv::KeyPoint kptInnerCurr = kptsCurr.at(it2->trainIdx);
            cv::KeyPoint kptInnerPrev = kptsPrev.at(it2->queryIdx);

            // compute distances and distance ratios
            double distCurr = cv::norm(kptOuterCurr.pt - kptInnerCurr.pt);
            double distPrev = cv::norm(kptOuterPrev.pt - kptInnerPrev.pt);

            if (distPrev > numeric_limits<double>::epsilon() && distCurr >= minDist){
                // avoid division by zero
                double distRatio = distCurr / distPrev;
                distRatios.push_back(distRatio);
            }
        } // eof inner loop over all matched kpts
    }   // eof outer loop over all matched kpts

    if (distRatios.size() == 0){
        TTC = NAN;
        return;
    }

    // computer camera-based TTC from distance ratios
    // mean
    double meanDistRatio = accumulate(distRatios.begin(), distRatios.end(), 0.0) / distRatios.size();
    // median
    sort(distRatios.begin(), distRatios.end());
    long medIndex = floor(distRatios.size()/2);
    double medDistRatio = distRatios.size() % 2 ==0 ? (distRatios[medIndex-1] + distRatios[medIndex])/2.0 : distRatios[medIndex];

    double dT = 1/frameRate;
    TTC = -dT/(1-medDistRatio);
}


void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC)
{
    double dT = 1/frameRate;    // time between two measurements in seconds
    double laneWidth = 4.0;     // assumed width of the ego lane

    vector<double> xPrev, xCurr;
    // calculate mean distance
    for (auto it1 = lidarPointsPrev.begin(); it1 != lidarPointsPrev.end(); ++it1){
        if (abs(it1->y) <= (laneWidth/2.0)){
            xPrev.push_back(it1->x);
        }
    }
    double xPrevMean = accumulate(xPrev.begin(), xPrev.end(), 0.0)/xPrev.size();

    for (auto it2 = lidarPointsCurr.begin(); it2 != lidarPointsCurr.end(); ++it2){
        if (abs(it2->y) <= (laneWidth/2.0)){
            xCurr.push_back(it2->x);
        }
    }

    double xCurrMean = accumulate(xCurr.begin(), xCurr.end(), 0.0)/xCurr.size();
    sort(xCurr.begin(), xCurr.end());    
    double xCurrMedian = xCurr[xCurr.size()/2];

    // TTC is defined as TTC = distance / velocity
    TTC = xCurrMean * dT / (xPrevMean - xCurrMean);
}


void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame, DataFrame &currFrame)
{
    int countBoxMatch[prevFrame.boundingBoxes.size()][currFrame.boundingBoxes.size()] = {0};
    for (auto itMatch = matches.begin(); itMatch != matches.end(); ++itMatch){
        int prevKptIdx = itMatch->queryIdx; // keypoint index in the previous frame
        int currKptIdx = itMatch->trainIdx; // keypoint index in the current frame

        cv::KeyPoint prevKpt = prevFrame.keypoints[prevKptIdx];
        cv::KeyPoint currKpt = currFrame.keypoints[currKptIdx];
        vector<int> prevBoxIDs, currBoxIDs;

        for (auto itBox1 = prevFrame.boundingBoxes.begin(); itBox1 != prevFrame.boundingBoxes.end(); ++itBox1){
            if (itBox1->roi.contains(prevKpt.pt)){
                prevBoxIDs.push_back(itBox1->boxID);
            }
        }

        for (auto itBox2 = currFrame.boundingBoxes.begin(); itBox2 != currFrame.boundingBoxes.end(); ++itBox2){
            if (itBox2->roi.contains(currKpt.pt)){
                currBoxIDs.push_back(itBox2->boxID);                
            }
        }

        for (auto prevBoxID:prevBoxIDs){
            for (auto currBoxID:currBoxIDs){
                countBoxMatch[prevBoxID][currBoxID]++;
            }
        }
    }

    for (auto itBox1 = prevFrame.boundingBoxes.begin(); itBox1 != prevFrame.boundingBoxes.end(); ++itBox1){
            int maxCount = 0;
            int currBestBoxID = 0;             
        for (auto itBox2 = currFrame.boundingBoxes.begin(); itBox2 != currFrame.boundingBoxes.end(); ++itBox2){
            int count = countBoxMatch[itBox1->boxID][itBox2->boxID];
            if (count > maxCount){
                maxCount = count;
                currBestBoxID = itBox2->boxID;
            }
        }
        bbBestMatches[itBox1->boxID] = currBestBoxID;
    }
}
