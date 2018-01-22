//
// Created by lucas on 21/01/18.
//

#include "wm_data_collector.h"

std::string _CAMERA_TOPIC;
std::string _YOLO_TOPIC;

int main(int argc, char **argv) {
	// Initialise ROS
	ros::init(argc, argv, "wm_data_collector");
	ros::NodeHandle n;

	// Get all parameters
	n.param("camera_topic", _CAMERA_TOPIC, std::string("/head_xtion/depth/image_raw"));
	ROS_INFO("camera_topic = %s", _CAMERA_TOPIC.c_str());
	n.param("yolo_topic", _YOLO_TOPIC, std::string("/darknet_ros/bounding_boxes"));
	ROS_INFO("yolo_topic = %s", _YOLO_TOPIC.c_str());

	// subscribe to the camera topic
	ros::Subscriber ImageSubscriber = n.subscribe(_CAMERA_TOPIC, 1, ImageListener);

	// subscribe to the yolo topic
	ros::Subscriber BoxesSubscriber = n.subscribe(_YOLO_TOPIC, 1, BoundingBoxListener);

	return 0;
}


void ImageListener(const sensor_msgs::ImageConstPtr& msg){

}

void BoundingBoxListener(darknet_ros_msgs::BoundingBoxes msg){

}