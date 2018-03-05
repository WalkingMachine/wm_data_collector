//
// Created by lucas on 21/01/18.
//

#include "wm_data_collector.h"

std::string _CAMERA_TOPIC;
std::string _YOLO_TOPIC;



DataCollector::DataCollector(int argc, char **argv) {
    // Initialise ROS
    ros::init(argc, argv, "wm_data_collector");
    ros::NodeHandle nh;

    // Get all parameters
    nh.param("camera_topic", _CAMERA_TOPIC, std::string("/camera/rgb/image_raw"));
    ROS_INFO("camera_topic = %s", _CAMERA_TOPIC.c_str());
    nh.param("yolo_topic", _YOLO_TOPIC, std::string("/darknet_ros/bounding_boxes"));
    ROS_INFO("yolo_topic = %s", _YOLO_TOPIC.c_str());

    // subscribe to the camera topic
    nh.subscribe(_CAMERA_TOPIC, 1, &DataCollector::ImageCallback, this);

    // subscribe to the yolo topic
    nh.subscribe(_YOLO_TOPIC, 1, &DataCollector::BoundingBoxCallback, this);


    colorClient = nh.serviceClient<wm_color_detector::AnalyseColor>("get_bounding_boxes_color");
    positionClient = nh.serviceClient<wm_frame_to_box::GetBoundingBoxes3D>("get_3d_bounding_boxes");

    // Waiting for events...
    ros::spin();

}

/**
 * Receive an image from camera and save them
 * @param msg 		The ros message
 */
void DataCollector::ImageCallback(sensor_msgs::ImageConstPtr msg) {
    Image = msg;
}

/**
 * Receive a list of bounding boxes and create entities from them
 * @param msg 		The ros message
 */
void DataCollector::BoundingBoxCallback(darknet_ros_msgs::BoundingBoxes msg) {
    // Convert from darknet format to sara format
    auto BoundingBoxes2D = ConvertBB(msg.boundingBoxes);


    // Convert the 2D bounding boxes into 3D ones
    wm_frame_to_box::GetBoundingBoxes3D BBService;
    BBService.request.boundingBoxes2D = BoundingBoxes2D;
    positionClient.call(BBService);
    auto BoundingBoxes3D = BBService.response.boundingBoxes3D;

    // Get the colors indentification
    wm_color_detector::AnalyseColor ColorService;
    ColorService.request.boundingBoxes = BoundingBoxes2D;
    ColorService.request.image = *Image;
    colorClient.call(ColorService);
    auto Colors = ColorService.response.colors;


	// Check if the number of elements matches
	if (Colors.size() != BoundingBoxes3D.size()) {
        ROS_ERROR("An error occurred with Color Recognition!");
    }


    // Create the entities from the bounding boxes
    std::vector<wm_data_collector::entity> Entities;
    int i{0};
    for (auto &boundingBox : BoundingBoxes3D) {
        wm_data_collector::entity en;
        en.BoundingBox = boundingBox;
        en.name = boundingBox.Class;
        en.position = boundingBox.Center;
        if (Colors.size() == BoundingBoxes3D.size())
            en.color = Colors[i].color;

        Entities.push_back(en);
        ++i;
    }


    	//TODO:Leg Detection Call
	//TODO:3Pose Call
	//TODO:Face Recognition Call
	//TODO:Gender Recognition Call

    // Print the entities list
	for (auto &ee : Entities)
		std::cout << ee;

}


/**
 * Receive a list of bounding boxes from Darknet and convert them to sara standard bounding boxes
 * @param DBBs 		Darknet bounding boxes
 * @return BBsOut 		sara bounding boxes
 */
std::vector<sara_msgs::BoundingBox2D> ConvertBB(std::vector<darknet_ros_msgs::BoundingBox> DBBs){
    std::vector<sara_msgs::BoundingBox2D> BBsOut;
    for (auto DBB : DBBs){
        sara_msgs::BoundingBox2D BBOut;
        BBOut.Class = DBB.Class;
        BBOut.probability = DBB.probability;
        BBOut.xmax = DBB.xmin;
        BBOut.xmin = DBB.xmin;
        BBOut.ymax = DBB.ymax;
        BBOut.ymin = DBB.ymin;
        BBsOut.push_back(BBOut);
    }
    return BBsOut;
}



int main(int argc, char **argv){

//    Data

}