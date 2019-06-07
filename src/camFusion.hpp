#pragma once

#include <stdio.h>
#include <vector>
#include <opencv2/core.hpp>
#include "dataStructures.h"
#include "kdtree.h"

void clusterLidarWithROI(std::vector<BoundingBox>& boundingBoxes, const std::vector<LidarPoint>& lidarPoints,
        float shrinkFactor, const cv::Mat& P_rect_xx, const cv::Mat& R_rect_xx, const cv::Mat& RT);

void matchBoundingBoxes(const std::vector<cv::DMatch>& matches, std::map<int, int>& bbBestMatches, const DataFrame& prevFrame, const DataFrame& currFrame);

void show3DObjects(const std::vector<BoundingBox>& boundingBoxes, const cv::Size& worldSize, const cv::Size& imageSize, bool bWait=true);

void clusterHelper(int index, const std::vector<std::vector<float>>& points, std::vector<int>& cluster, std::vector<bool>& processed,
        const std::shared_ptr<KdTree>& tree, float distanceTol);

std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>>& points, const std::shared_ptr<KdTree>& tree, float distanceTol);

void computeTTCLidar(const std::vector<LidarPoint>& lidarPointsPrev,
                     const std::vector<LidarPoint>& lidarPointsCurr, double frameRate, double& TTC);

void clusterKptMatchesWithROI(BoundingBox& boundingBox, const std::vector<cv::KeyPoint>& kptsPrev,
        const std::vector<cv::KeyPoint>& kptsCurr, const std::vector<cv::DMatch>& kptMatches);

void computeTTCCamera(const std::vector<cv::KeyPoint>& kptsPrev, const std::vector<cv::KeyPoint>& kptsCurr,
                      const std::vector<cv::DMatch>& kptMatches, double frameRate, double& TTC);

