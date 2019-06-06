#include <fstream>
#include <sstream>
#include <iostream>

#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include "objectDetection2D.hpp"


using namespace std;

// detects objects in an image using the YOLO library and a set of pre-trained objects from the COCO database;
// a set of 80 classes is listed in "coco.names" and pre-trained weights are stored in "yolov3.weights"
void detectObjects(const cv::Mat& img, std::vector<BoundingBox>& bBoxes, float confThreshold, float nmsThreshold,
                   const std::string& basePath, const std::string& classesFile,
                   const std::string& modelConfiguration, const std::string& modelWeights, bool bVis) {

    // TODO: load class name in main function
    // load class names from file
    vector<string> classes;
    ifstream ifs(classesFile.c_str());
    string line;
    while (getline(ifs, line)) {
        classes.push_back(line);
    }

    // load neural network
    cv::dnn::Net net = cv::dnn::readNetFromDarknet(modelConfiguration, modelWeights);
    net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
    net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
    
    // generate 4D blob from input image
    cv::Mat blob;
    vector<cv::Mat> netOutput;
    double scalefactor = 1 / 255.0;
    cv::Size size = cv::Size(416, 416);
    cv::Scalar mean = cv::Scalar(0,0,0);
    bool swapRB = false;
    bool crop = false;
    cv::dnn::blobFromImage(img, blob, scalefactor, size, mean, swapRB, crop);
    
    // Get names of output layers
    vector<cv::String> names;
    vector<int> outLayers = net.getUnconnectedOutLayers(); // get indices of  output layers, i.e. layers with unconnected outputs
    vector<cv::String> layersNames = net.getLayerNames(); // get names of all layers in the network
    
    names.resize(outLayers.size());
    for (size_t i = 0; i < outLayers.size(); i++) {
        // Get the names of the output layers in names
        names[i] = layersNames[outLayers[i] - 1];
    }
    
    // invoke forward propagation through network
    net.setInput(blob);
    net.forward(netOutput, names);
    
    // Scan through all bounding boxes and keep only the ones with high confidence
    std::vector<int> classIds;
    std::vector<float> confidences;
    std::vector<cv::Rect> boxes;

    for (auto & i : netOutput) {
        auto* data = (float*)i.data;
        for (int j = 0; j < i.rows; j++, data += i.cols) {
            cv::Mat scores = i.row(j).colRange(5, i.cols);
            cv::Point classId;
            double confidence;
            
            // Get the value and location of the maximum score
            cv::minMaxLoc(scores, nullptr, &confidence, nullptr, &classId);
            if (confidence > confThreshold) {
                cv::Rect box;
                int cx, cy;
                cx = static_cast<int>(data[0] * img.cols);
                cy = static_cast<int>(data[1] * img.rows);
                box.width = static_cast<int>(data[2] * img.cols);
                box.height = static_cast<int>(data[3] * img.rows);
                box.x = cx - box.width/2; // left
                box.y = cy - box.height/2; // top
                
                boxes.push_back(box);
                classIds.push_back(classId.x);
                confidences.push_back(static_cast<float>(confidence));
            }
        }
    }
    
    // perform non-maximum suppression
    vector<int> indices;
    cv::dnn::NMSBoxes(boxes, confidences, confThreshold, nmsThreshold, indices);
    for(int & index : indices) {
        BoundingBox bBox;
        bBox.roi = boxes[index];
        bBox.classID = classIds[index];
        bBox.confidence = confidences[index];
        bBox.boxID = static_cast<int>(bBoxes.size()); // zero-based unique identifier for this bounding box
        bBoxes.push_back(bBox);
    }
    
    // show results
    if(bVis) {
        cv::Mat visImg = img.clone();
        for(auto & bBox : bBoxes) {
            // Draw rectangle displaying the bounding box
            int top, left, width, height;
            top = bBox.roi.y;
            left = bBox.roi.x;
            width = bBox.roi.width;
            height = bBox.roi.height;
            cv::rectangle(visImg, cv::Point(left, top), cv::Point(left+width, top+height),cv::Scalar(0, 255, 0), 2);

            string label;
            label.append(classes[(bBox.classID)]).append(std::to_string(bBox.boxID)).append(":").append(cv::format("%.2f", bBox.confidence));
        
            // Display label at the top of the bounding box
            int baseLine;
            cv::Size labelSize = getTextSize(label, cv::FONT_ITALIC, 0.5, 1, &baseLine);
            top = max(top, labelSize.height);
            rectangle(visImg, cv::Point(left, top - round(1.5*labelSize.height)), cv::Point(left + round(labelSize.width), top + baseLine), cv::Scalar(255, 255, 255), cv::FILLED);
            cv::putText(visImg, label, cv::Point(left, top), cv::FONT_ITALIC, 0.5, cv::Scalar(0,0,0),1);
        }
        
        string windowName = "Object classification";
        cv::namedWindow( windowName, 1 );
        cv::imshow( windowName, visImg );
//        cv::waitKey(0); // wait for key to be pressed
    }
}
