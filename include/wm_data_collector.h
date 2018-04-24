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
#include "people_msgs/PositionMeasurementArray.h"
#include "visualization_msgs/Marker.h"
#include <sstream>


std::string _CAMERA_TOPIC;
std::string _DEPTH_CAMERA_TOPIC;
std::string _YOLO_TOPIC;
std::string _ENTITIES;
std::string _PEOPLE_TOPIC;
std::string _LEGS_TOPIC;
std::string _MARKER_TOPIC;
double _LEG_DISTANCE;
double _LEG_TOLERANCE;
double _CAMERA_TOLERANCE;
double _PEOPLE_TOLERANCE;
double _ENTITY_DECAY;
double _CUMULATION;
double _ENTITY_FRICTION;
double _NAME_WEIGHT;
double _PERSON_WEIGHT;
double _COLOR_WEIGHT;
double _GENDER_WEIGHT;
double _THRESHOLD;
bool _NORMALISE;
double _MAX_PROBABILITY;
double _SPEED_RATIO;
double _LEG_PROBABILITY_RATIO;

class DataCollector {

    ros::ServiceClient colorClient;
    ros::ServiceClient positionClient;
    ros::Publisher entityPublisher;
    ros::Publisher peoplePublisher;
    ros::Publisher markerPublisher;

    sensor_msgs::ImageConstPtr Image;
    sensor_msgs::ImageConstPtr DepthImage;

    sara_msgs::Entities Entities;

    int ProceduralID;

    // Receive an image from camera
    void ImageCallback(sensor_msgs::ImageConstPtr msg);
    // Receive a depth image from camera
    void DepthImageCallback(sensor_msgs::ImageConstPtr msg);
    // Receive the Bounding Boxes from Yolo and process then
    void LegsCallback(people_msgs::PositionMeasurementArray msg);
    // Receive the Bounding Boxes from Yolo and process them
    void BoundingBoxCallback(darknet_ros_msgs::BoundingBoxes msg);

    // Update all entities with the new data received
    void AddEntity(sara_msgs::Entity newEntity, double tolerance);

    // Update entities
    void UpdateEntities();

    // Publish the visual clues onto markers for rviz
    void PublishVisualisation();

    // Compare two entities and return their difference level
    double CompareEntities(sara_msgs::Entity &en1, sara_msgs::Entity &en2, double MaxDistance);

    // Merge two entities into one and return the result
    void MergeEntities(sara_msgs::Entity &en1, sara_msgs::Entity &en2);

public:

    DataCollector(int argc, char **argv);

};

std::vector<sara_msgs::BoundingBox2D> ConvertBB(std::vector<darknet_ros_msgs::BoundingBox>);

#endif //PROJECT_WM_DATA_COLLECTOR_NODE_H
