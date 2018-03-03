//
// Created by lucas on 21/01/18.
//

#ifndef PROJECT_WM_DATA_COLLECTOR_NODE_H
#define PROJECT_WM_DATA_COLLECTOR_NODE_H

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include "darknet_ros_msgs/BoundingBoxes.h"
#include "wm_data_collector/BoundingBox.h"
#include "wm_color_detector/AnalyseColor.h"
#include "EntityWraper.h"

#include "CDataEntity.h"

#include <sstream>

// Receive an image from camera
void ImageListener(sensor_msgs::ImageConstPtr msg);

// Receive the Bounding Boxes from Yolo and process then
void BoundingBoxListener(darknet_ros_msgs::BoundingBoxes msg);

#endif //PROJECT_WM_DATA_COLLECTOR_NODE_H
