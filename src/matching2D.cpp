#include <numeric>
#include "matching2D.hpp"

using namespace std;

// Find best matches for keypoints in two camera images based on several matching methods
void matchDescriptors(std::vector<cv::KeyPoint> &kPtsSource, std::vector<cv::KeyPoint> &kPtsRef, cv::Mat &descSource, cv::Mat &descRef,
                      std::vector<cv::DMatch> &matches, const std::string& descriptorType, const std::string& matcherType, const std::string& selectorType) {
    // configure matcher
    bool crossCheck = false;
    cv::Ptr<cv::DescriptorMatcher> matcher;

    if ("MAT_BF" == matcherType) {
        int normType = (descriptorType == "DES_BINARY") ? cv::NORM_HAMMING : cv::NORM_L2;
        matcher = cv::BFMatcher::create(normType, crossCheck);
    }
    else if ("MAT_FLANN" == matcherType) {
        if (descSource.type() != CV_32F) {
            // OpenCV bug workaround : convert binary descriptors to floating point due to a bug in current OpenCV implementation
            descSource.convertTo(descSource, CV_32F);
            descRef.convertTo(descRef, CV_32F);
        }

        matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
    }

    // perform matching task
    if ("SEL_NN" == selectorType) {
        // nearest neighbor (best match)
        matcher->match(descSource, descRef, matches); // Finds the best match for each descriptor in desc1
    }
    else if ("SEL_KNN" == selectorType) {
        // k nearest neighbors (k=2)
        vector<vector<cv::DMatch>> knn_matches;
        matcher->knnMatch(descSource, descRef, knn_matches, 2); // finds the 2 best matches

        // filter matches using descriptor distance ratio test
        double minDescDistRatio = 0.8;
        for (auto & knn_match : knn_matches) {
            if (knn_match[0].distance < minDescDistRatio * knn_match[1].distance) {
                matches.push_back(knn_match[0]);
            }
        }
    }

    cout << "Number of matched keypoints = " << matches.size() << endl;
}

// Use one of several types of state-of-art descriptors to uniquely identify keypoints
double descKeypoints(vector<cv::KeyPoint> &keypoints, cv::Mat &img, cv::Mat &descriptors, const string& descriptorType) {
    // select appropriate descriptor  FREAK,
    cv::Ptr<cv::DescriptorExtractor> extractor;
    if ("BRISK" == descriptorType) {
        int threshold = 30;        // FAST/AGAST detection threshold score.
        int octaves = 3;           // detection octaves (use 0 to do single scale)
        float patternScale = 1.0f; // apply this scale to the pattern used for sampling the neighbourhood of a keypoint.

        extractor = cv::BRISK::create(threshold, octaves, patternScale);
    }
    else if ("BRIEF" == descriptorType) {
        extractor = cv::xfeatures2d::BriefDescriptorExtractor::create();
    }
    else if ("ORB" == descriptorType) {
        extractor = cv::ORB::create();
    }
    else if ("FREAK" == descriptorType) {
        extractor = cv::xfeatures2d::FREAK::create();
    }
    else if ("AKAZE" == descriptorType) {
        extractor = cv::AKAZE::create();
    }
    else if ("SIFT" == descriptorType){
        extractor = cv::xfeatures2d::SIFT::create();
    }
    else {
        std::cerr << "Invalid detector type!!!" << std::endl;
    }

    // perform feature description
    auto t = (double)cv::getTickCount();
    extractor->compute(img, keypoints, descriptors);
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    cout << descriptorType << " descriptor extraction in " << 1000 * t / 1.0 << " ms" << endl;

    return t;
}


// Detect keypoints in image using the traditional Shi-Thomasi detector
double detKeypointsShiTomasi(vector<cv::KeyPoint> &keypoints, const cv::Mat &img) {
    // compute detector parameters based on image size
    int blockSize = 4;       //  size of an average block for computing a derivative covariation matrix over each pixel neighborhood
    double maxOverlap = 0.0; // max. permissible overlap between two features in %
    double minDistance = (1.0 - maxOverlap) * blockSize;
    int maxCorners = img.rows * img.cols / max(1.0, minDistance); // max. num. of keypoints

    double qualityLevel = 0.01; // minimal accepted quality of image corners
    double k = 0.04;

    // Apply corner detection
    auto t = (double)cv::getTickCount();
    vector<cv::Point2f> corners;
    cv::goodFeaturesToTrack(img, corners, maxCorners, qualityLevel, minDistance, cv::Mat(), blockSize, false, k);

    // add corners to result vector
    for (auto & corner : corners) {
        cv::KeyPoint newKeyPoint;
        newKeyPoint.pt = cv::Point2f(corner.x, corner.y);
        newKeyPoint.size = blockSize;
        keypoints.push_back(newKeyPoint);
    }

    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    cout << "Shi-Tomasi detection with n = " << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;

    return t;
}


// Harris Corner
double detKeypointsHarris(std::vector<cv::KeyPoint> &keypoints, const cv::Mat &img) {
    // Detector parameters
    int blockSize = 2; // for every pixel, a blockSize × blockSize neighborhood is considered
    int apertureSize = 3; // aperture parameter for Sobel operator (must be odd)
    int minResponse = 100; // minimum value for a corner in the 8bit scaled response matrix
    double k = 0.04; // Harris parameter

    auto t = (double)cv::getTickCount();
    // Detect Harris corners and normalize output
    cv::Mat dst, dst_norm, dst_norm_scaled;
    dst = cv::Mat::zeros(img.size(), CV_32FC1);
    cv::cornerHarris(img, dst, blockSize, apertureSize, k, cv::BORDER_DEFAULT);
    cv::normalize( dst, dst_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat() );
    cv::convertScaleAbs( dst_norm, dst_norm_scaled );

    // Look for prominent corners and instantiate keypoints
    // max. permissible overlap between two features in %, used during non-maxima suppression
    double maxOverlap = 0.0;
    for (size_t j = 0; j < dst_norm.rows; j++) {
        for (size_t i = 0; i < dst_norm.cols; i++) {
            int response = (int)dst_norm.at<float>(j, i);
            if (response > minResponse) {
                // only store points above a threshold

                cv::KeyPoint newKeyPoint;
                newKeyPoint.pt = cv::Point2f(i, j);
                newKeyPoint.size = 2 * apertureSize;
                newKeyPoint.response = response;

                // perform non-maximum suppression (NMS) in local neighbourhood around new key point
                bool bOverlap = false;
                for (auto & keypoint : keypoints) {
                    double kptOverlap = cv::KeyPoint::overlap(newKeyPoint, keypoint);
                    if (kptOverlap > maxOverlap) {
                        bOverlap = true;
                        if (newKeyPoint.response > keypoint.response) {
                            // if overlap is >t AND response is higher for new kpt
                            keypoint = newKeyPoint; // replace old key point with new one
                            break;             // quit loop over keypoints
                        }
                    }
                }
                if (!bOverlap) {
                    // only add new key point if no overlap has been found in previous NMS
                    keypoints.push_back(newKeyPoint); // store new keypoint in dynamic list
                }
            }
        } // eof loop over cols
    }     // eof loop over rows

    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    cout << "Harris Corner detection with n = " << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;

    return t;
}


// FAST, BRISK, ORB, AKAZE, SIFT
double detKeypointsModern(std::vector<cv::KeyPoint> &keypoints, const cv::Mat &img, const std::string& detectorType) {
    cv::Ptr<cv::FeatureDetector> detector;
    if ("FAST" == detectorType) {
        int threshold = 30;                                                              // difference between intensity of the central pixel and pixels of a circle around this pixel
        bool bNMS = true;                                                                // perform non-maxima suppression on keypoints
        cv::FastFeatureDetector::DetectorType type = cv::FastFeatureDetector::TYPE_9_16; // TYPE_9_16, TYPE_7_12, TYPE_5_8
        detector = cv::FastFeatureDetector::create(threshold, bNMS, type);
    }
    else if ("BRISK" == detectorType) {
        detector = cv::BRISK::create();
    }
    else if ("ORB" == detectorType) {
        detector = cv::ORB::create();
    }
    else if ("AKAZE" == detectorType) {
        detector = cv::AKAZE::create();
    }
    else if ("SIFT" == detectorType){
        detector = cv::xfeatures2d::SIFT::create();
    }
    else {
        std::cerr << "Invalid detector type!!!" << std::endl;
    }

    auto t = (double)cv::getTickCount();
    detector->detect(img, keypoints);
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    cout << detectorType << " detection with n = " << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;

    return t;
}