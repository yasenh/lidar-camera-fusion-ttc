
#include <iostream>
#include <algorithm>
#include <numeric>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "camFusion.hpp"
#include "dataStructures.h"

using namespace std;


// Create groups of LiDAR points whose projection into the camera falls into the same bounding box
void clusterLidarWithROI(std::vector<BoundingBox> &boundingBoxes, const std::vector<LidarPoint>& lidarPoints,
        float shrinkFactor, const cv::Mat& P_rect_xx, const cv::Mat& R_rect_xx, const cv::Mat& RT) {
    // loop over all LiDAR points and associate them to a 2D bounding box
    cv::Mat X(4, 1, cv::DataType<double>::type);
    cv::Mat Y(3, 1, cv::DataType<double>::type);

    for (auto & lidarPoint : lidarPoints) {
        // assemble vector for matrix-vector-multiplication
        X.at<double>(0, 0) = lidarPoint.x;
        X.at<double>(1, 0) = lidarPoint.y;
        X.at<double>(2, 0) = lidarPoint.z;
        X.at<double>(3, 0) = 1;

        // project LiDAR point into camera
        Y = P_rect_xx * R_rect_xx * RT * X;
        cv::Point pt;
        pt.x = static_cast<int>(Y.at<double>(0, 0) / Y.at<double>(0, 2)); // pixel coordinates
        pt.y = static_cast<int>(Y.at<double>(1, 0) / Y.at<double>(0, 2));

        vector<vector<BoundingBox>::iterator> enclosingBoxes; // pointers to all bounding boxes which enclose the current LiDAR point
        for (auto it2 = boundingBoxes.begin(); it2 != boundingBoxes.end(); ++it2) {
            // shrink current bounding box slightly to avoid having too many outlier points around the edges
            cv::Rect smallerBox;
            smallerBox.x = static_cast<int>(it2->roi.x + shrinkFactor * it2->roi.width / 2.0);
            smallerBox.y = static_cast<int>(it2->roi.y + shrinkFactor * it2->roi.height / 2.0);
            smallerBox.width = static_cast<int>(it2->roi.width * (1 - shrinkFactor));
            smallerBox.height = static_cast<int>(it2->roi.height * (1 - shrinkFactor));

            // check weather point is within current bounding box
            if (smallerBox.contains(pt)) {
                enclosingBoxes.push_back(it2);
            }

        } // eof loop over all bounding boxes

        // check weather point has been enclosed by one or by multiple boxes
        if (1 == enclosingBoxes.size()) {
            // add LiDAR point to bounding box
            enclosingBoxes[0]->lidarPoints.push_back(lidarPoint);
        }
    } // eof loop over all LiDAR points
}


void show3DObjects(const std::vector<BoundingBox> &boundingBoxes, const cv::Size& worldSize, const cv::Size& imageSize, bool bWait) {
    // create topview image
    cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(0, 0, 0));

    for(const auto& boundingBox : boundingBoxes) {
        // create randomized color for current 3D object
        cv::RNG rng(boundingBox.boxID);
        cv::Scalar currColor = cv::Scalar(rng.uniform(0,150), rng.uniform(0, 150), rng.uniform(0, 150));

        // plot LiDAR points into top view image
        int top = 1e8, left = 1e8, bottom = 0, right = 0;
        float xwmin = 1e8, ywmin = 1e8, ywmax = -1e8;
        for (const auto & lidarPoint : boundingBox.lidarPoints) {
            // world coordinates
            float xw = lidarPoint.x; // world position in m with x facing forward from sensor
            float yw = lidarPoint.y; // world position in m with y facing left from sensor

            xwmin = xwmin < xw ? xwmin : xw;
            ywmin = ywmin < yw ? ywmin : yw;
            ywmax = ywmax > yw ? ywmax : yw;

            // top-view coordinates
            int x = static_cast<int>((-yw * imageSize.width / worldSize.width) + imageSize.width / 2.0f);
            int y = static_cast<int>((-xw * imageSize.height / worldSize.height) + imageSize.height);


            // find enclosing rectangle
            top = top < y ? top : y;
            left = left < x ? left : x;
            bottom = bottom > y ? bottom : y;
            right = right > x ? right : x;

            // draw individual point
            cv::circle(topviewImg, cv::Point(x, y), 4, currColor, -1);
        }

        // draw enclosing rectangle
        cv::rectangle(topviewImg, cv::Point(left, top), cv::Point(right, bottom),cv::Scalar(0,0,255), 2);

        // augment object with some key data
        char str1[200], str2[200];
        sprintf(str1, "id = %d, #pts = %d", boundingBox.boxID, (int)boundingBox.lidarPoints.size());
        putText(topviewImg, str1, cv::Point2f(left-250, bottom+50), cv::FONT_ITALIC, 1, currColor);
        sprintf(str2, "xmin = %2.2f m, yw = %2.2f m", xwmin, ywmax - ywmin);
        putText(topviewImg, str2, cv::Point2f(left-250, bottom+125), cv::FONT_ITALIC, 1, currColor);
    }

    // plot distance markers
    float lineSpacing = 2.0; // gap between distance markers
    int nMarkers = floor(worldSize.height / lineSpacing);
    for (size_t i = 0; i < nMarkers; ++i) {
        int y = static_cast<int>((-(i * lineSpacing) * imageSize.height / worldSize.height) + imageSize.height);
        cv::line(topviewImg, cv::Point(0, y), cv::Point(imageSize.width, y), cv::Scalar(255, 0, 0));
    }

    // display image
    string windowName = "3D Objects";
    cv::namedWindow(windowName, 1);
    cv::imshow(windowName, topviewImg);

    if(bWait) {
        cv::waitKey(0); // wait for key to be pressed
    }
}


// associate a given bounding box with the keypoints it contains
void clusterKptMatchesWithROI(BoundingBox &boundingBox, std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, std::vector<cv::DMatch> &kptMatches) {
    // ...
}


// Compute time-to-collision (TTC) based on keypoint correspondences in successive images
void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, 
                      std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC, cv::Mat *visImg) {
    // ...
}


void computeTTCLidar(const std::vector<LidarPoint> &lidarPointsPrev,
                     const std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC) {
    // auxiliary variables
    double dT = 1 / frameRate;        // time between two measurements in seconds
    constexpr double laneWidth = 4.0; // assumed width of the ego lane

    // find closest distance to LiDAR points within ego lane
    double minXPrev = 1e9, minXCurr = 1e9;
    for (const auto & it : lidarPointsPrev) {
        if (abs(it.y) <= laneWidth / 2.0) { // 3D point within ego lane?
            minXPrev = it.x < minXPrev ? it.x : minXPrev;
        }
    }

    for (const auto & it : lidarPointsCurr) {
        if (abs(it.y) <= laneWidth / 2.0) { // 3D point within ego lane?
            minXCurr = it.x < minXCurr ? it.x : minXCurr;
        }
    }

    // compute TTC from both measurements
    TTC = minXCurr * dT / (minXPrev - minXCurr);
}


template<typename KeyType, typename ValueType>
std::pair<KeyType, ValueType> get_max(const std::map<KeyType, ValueType>& x) {
    using pairtype = std::pair<KeyType, ValueType>;
    return *std::max_element(x.begin(), x.end(), [] (const pairtype & p1, const pairtype & p2) {
        return p1.second < p2.second;
    });
}


void matchBoundingBoxes(const std::vector<cv::DMatch>& matches, std::map<int, int>& bbBestMatches, const DataFrame& prevFrame, const DataFrame& currFrame) {
    for (const auto& prevBox : prevFrame.boundingBoxes) {
        std::map<int, int> m;
        for (const auto& currBox : currFrame.boundingBoxes) {

            // queryIdx refers to keypoints1 and trainIdx refers to keypoints2
            for (const auto &match : matches) {
                const auto &prevKeyPoint = prevFrame.keypoints[match.queryIdx].pt;
                if (prevBox.roi.contains(prevKeyPoint)) {
                    const auto &currKeyPoint = currFrame.keypoints[match.trainIdx].pt;
                    if (currBox.roi.contains(currKeyPoint)) {
                        if(0 == m.count(currBox.boxID)) {
                            m[currBox.boxID] = 1;
                        }
                        else {
                            m[currBox.boxID]++;
                        }
                    }
                }
            } // eof iterating all matches
        } // eof iterating all current bounding boxes

        auto max=get_max(m);

        bbBestMatches[prevBox.boxID] = max.first;
        std::cout << "ID Matching: " << prevBox.boxID << "=>" << max.first << std::endl;

    } // eof iterating all previous bounding boxes
}
