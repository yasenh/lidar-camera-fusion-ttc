
#ifndef objectDetection2D_hpp
#define objectDetection2D_hpp

#include <stdio.h>
#include <opencv2/core.hpp>

#include "dataStructures.h"

void detectObjects(const cv::Mat& img, std::vector<BoundingBox>& bBoxes, float confThreshold, float nmsThreshold,
                   const std::string& basePath, const std::string& classesFile,
                   const std::string& modelConfiguration, const std::string& modelWeights, bool bVis);

#endif /* objectDetection2D_hpp */
