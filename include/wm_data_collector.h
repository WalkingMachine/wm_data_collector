//
// Created by lucas on 21/01/18.
//

#ifndef PROJECT_WM_DATA_COLLECTOR_NODE_H
#define PROJECT_WM_DATA_COLLECTOR_NODE_H

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include "wm_color_detector/AnalyseColor.h"
#include "darknet_ros_msgs/BoundingBoxes.h"
#include "sara_msgs/BoundingBox2D.h"
#include "wm_frame_to_box/GetBoundingBoxes3D.h"
#include "sara_msgs/Entity.h"
#include "sara_msgs/Entities.h"


#include <sstream>

class DataCollector {

    ros::ServiceClient colorClient;
    ros::ServiceClient positionClient;
    sensor_msgs::ImageConstPtr Image;
    sensor_msgs::ImageConstPtr DepthImage;
    ros::Publisher entityPublisher;

// Receive an image from camera
    void ImageCallback(sensor_msgs::ImageConstPtr msg);
// Receive a depth image from camera
    void DepthImageCallback(sensor_msgs::ImageConstPtr msg);


    // Receive the Bounding Boxes from Yolo and process then
    void BoundingBoxCallback(darknet_ros_msgs::BoundingBoxes msg);

public:

    DataCollector(int argc, char **argv);

};

std::vector<sara_msgs::BoundingBox2D> ConvertBB(std::vector<darknet_ros_msgs::BoundingBox>);

#endif //PROJECT_WM_DATA_COLLECTOR_NODE_H
