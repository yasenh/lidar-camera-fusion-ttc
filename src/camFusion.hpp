#pragma once

#include <stdio.h>
#include <vector>
#include <opencv2/core.hpp>
#include "dataStructures.h"


void clusterLidarWithROI(std::vector<BoundingBox>& boundingBoxes, const std::vector<LidarPoint>& lidarPoints,
        float shrinkFactor, const cv::Mat& P_rect_xx, const cv::Mat& R_rect_xx, const cv::Mat& RT);

void clusterKptMatchesWithROI(BoundingBox &boundingBox, std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, std::vector<cv::DMatch> &kptMatches);

void matchBoundingBoxes(const std::vector<cv::DMatch>& matches, std::map<int, int>& bbBestMatches, const DataFrame& prevFrame, const DataFrame& currFrame);

void show3DObjects(std::vector<BoundingBox> &boundingBoxes, const cv::Size& worldSize, const cv::Size& imageSize, bool bWait=true);

void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr,
                      std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC, cv::Mat *visImg=nullptr);

void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC);
