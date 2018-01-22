//
// Created by lucas on 21/01/18.
//

#include "wm_data_collector.h"

sensor_msgs::ImageConstPtr lastImage;
ros::ServiceClient client;

std::string _CAMERA_TOPIC;
std::string _YOLO_TOPIC;

int main(int argc, char **argv) {
	// Initialise ROS
	ros::init(argc, argv, "wm_data_collector");
	ros::NodeHandle n;

	// Get all parameters
	n.param("camera_topic", _CAMERA_TOPIC, std::string("/camera/rgb/image_raw"));
	ROS_INFO("camera_topic = %s", _CAMERA_TOPIC.c_str());
	n.param("yolo_topic", _YOLO_TOPIC, std::string("/darknet_ros/bounding_boxes"));
	ROS_INFO("yolo_topic = %s", _YOLO_TOPIC.c_str());

	// subscribe to the camera topic
	ros::Subscriber ImageSubscriber = n.subscribe(_CAMERA_TOPIC, 1, ImageListener);

	// subscribe to the yolo topic
	ros::Subscriber BoxesSubscriber = n.subscribe(_YOLO_TOPIC, 1, BoundingBoxListener);


	client = n.serviceClient<wm_color_detector::AnalyseColor>("get_bounding_boxes_color");

	// Waiting for events...
	ros::spin();

	return 0;
}

/**
 * Receive an image from camera and save them
 * @param msg 		The ros message
 */
void ImageListener(sensor_msgs::ImageConstPtr msg) {
	lastImage = msg;
}

void BoundingBoxListener(darknet_ros_msgs::BoundingBoxes msg){
	ROS_INFO("BoundingBoxes received! (%i)", (int) msg.boundingBoxes.size());

	sensor_msgs::ImageConstPtr image = lastImage;

	std::vector<CDataEntity> dataEntities;
	std::vector<wm_color_detector::BoundingBox> boxes;

	for (auto &boundingBox : msg.boundingBoxes) {

		CDataEntity anEntity(boundingBox.xmin, boundingBox.ymin, boundingBox.xmax, boundingBox.ymax, boundingBox.Class);

		if (anEntity.isBoundingBoxInitialised()) {
			dataEntities.push_back(anEntity);
			boxes.push_back(anEntity.getBox());
		} else {
			ROS_WARN("Bad Bounding Box coordinate given! (%s in %ix%i to %ix%i)", boundingBox.Class.c_str(),
			         boundingBox.xmin, boundingBox.ymin,
			         boundingBox.xmax, boundingBox.ymax);
			ROS_WARN("Bounding Box will be ignored.");
		}
	}

	// Color Recognition Call
	wm_color_detector::AnalyseColor srv;
	srv.request.boundingBoxes = boxes;
	srv.request.image = *image;

	// If the service work and the number of returned element is good
	if (client.call(srv) && srv.response.colors.size() == dataEntities.size()) {
		for (int index = 0; index < dataEntities.size(); index++) {
			dataEntities[index].setColor(srv.response.colors[index].color);
		}
	} else {
		ROS_WARN("An error occurred with Color Recognition!");
	}

	//TODO:Leg Detection Call
	//TODO:3Pose Call
	//TODO:Face Recognition Call
	//TODO:Gender Recognition Call

	// Print the entities list
	for (auto &dataEntity : dataEntities) {
		dataEntity.printEntity();
	}
}