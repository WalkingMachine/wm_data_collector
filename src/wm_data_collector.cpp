//
// Created by lucas on 21/01/18.
//

#include "wm_data_collector.h"

std::string _CAMERA_TOPIC;
std::string _DEPTH_CAMERA_TOPIC;
std::string _YOLO_TOPIC;
std::string _ENTITIES;
std::string _PEOPLE_TOPIC;
std::string _LEGS_TOPIC;
double _LEG_DISTANCE;


DataCollector::DataCollector(int argc, char **argv) {
    // Initialise ROS
    ros::init(argc, argv, "wm_data_collector");
    ros::NodeHandle nh;

    // Get all parameters
    nh.param("camera_topic", _CAMERA_TOPIC, std::string("/head/xtion/rgb/image_raw"));
    ROS_INFO("camera_topic = %s", _CAMERA_TOPIC.c_str());
    nh.param("depth_camera_topic", _DEPTH_CAMERA_TOPIC, std::string("/head/xtion/depth/image_raw"));
    ROS_INFO("depth_camera_topic = %s", _DEPTH_CAMERA_TOPIC.c_str());
    nh.param("yolo_topic", _YOLO_TOPIC, std::string("/darknet_ros/bounding_boxes"));
    ROS_INFO("yolo_topic = %s", _YOLO_TOPIC.c_str());
    nh.param("entities_topic", _ENTITIES, std::string("/entities"));
    ROS_INFO("entities_topic = %s", _ENTITIES.c_str());
    nh.param("people_topic", _PEOPLE_TOPIC, std::string("/people"));
    ROS_INFO("people_topic = %s", _PEOPLE_TOPIC.c_str());
    nh.param("legs_topic", _LEGS_TOPIC, std::string("/legs"));
    ROS_INFO("legs_topic = %s", _LEGS_TOPIC.c_str());
    nh.param("leg_distance", _LEG_DISTANCE, 1.0);
    ROS_INFO("leg_distance = %lf", _LEG_DISTANCE);

    // Subscribers
    ros::Subscriber camera_sub = nh.subscribe(_CAMERA_TOPIC, 1, &DataCollector::ImageCallback, this);
    ros::Subscriber depth_camera_sub = nh.subscribe(_DEPTH_CAMERA_TOPIC, 1, &DataCollector::DepthImageCallback, this);
    ros::Subscriber yolo_sub = nh.subscribe(_YOLO_TOPIC, 1, &DataCollector::BoundingBoxCallback, this);
    ros::Subscriber leg_sub = nh.subscribe(_LEGS_TOPIC, 1, &DataCollector::LegsCallback, this);

    // Service clients
    colorClient = nh.serviceClient<wm_color_detector::AnalyseColor>("get_bounding_boxes_color");
    positionClient = nh.serviceClient<wm_frame_to_box::GetBoundingBoxes3D>("get_3d_bounding_boxes");

    // Publishers
    entityPublisher = nh.advertise<sara_msgs::Entities>(_ENTITIES, 10);
    peoplePublisher = nh.advertise<people_msgs::PositionMeasurementArray>(_PEOPLE_TOPIC, 10);

    ROS_INFO("running");
    // Waiting for events...
    ros::spin();

}

/**
 * Receive an image from camera and save it
 * @param msg 		The ros message
 */
void DataCollector::ImageCallback(sensor_msgs::ImageConstPtr msg) {
//    ROS_INFO("image received");
    Image = msg;
}

/**
 * Receive a depth image from camera and save it
 * @param msg 		The ros message
 */
void DataCollector::DepthImageCallback(sensor_msgs::ImageConstPtr msg) {
//    ROS_INFO("image received");
    DepthImage = msg;
}

/**
 * Receive a list of legs and store them for later
 * @param msg 		The ros message
 */
void DataCollector::LegsCallback(people_msgs::PositionMeasurementArray msg) {
//    ROS_INFO("Legs received");
    Legs = msg;
}

/**
 * Receive a list of bounding boxes and create entities from them
 * @param msg 		The ros message
 */
void DataCollector::BoundingBoxCallback(darknet_ros_msgs::BoundingBoxes msg) {

//    ROS_INFO("BB received");
    // Convert from darknet format to sara format
    auto BoundingBoxes2D = ConvertBB(msg.boundingBoxes);


    // Convert the 2D bounding boxes into 3D ones
    wm_frame_to_box::GetBoundingBoxes3D BBService;
    BBService.request.boundingBoxes2D = BoundingBoxes2D;
    BBService.request.image = *DepthImage;
    positionClient.call(BBService);
    auto BoundingBoxes3D = BBService.response.boundingBoxes3D;

    // Get the colors indentification
    wm_color_detector::AnalyseColor ColorService;
    ColorService.request.boundingBoxes = BoundingBoxes2D;
    ColorService.request.image = *Image;
    colorClient.call(ColorService);
    auto Colors = ColorService.response.colors;


//    ROS_INFO("comparing %d and %d", Colors.size(), BoundingBoxes3D.size());
    // Check if the number of elements matches
    if (Colors.size() != BoundingBoxes3D.size()) {
        ROS_ERROR("An error occurred with Color Recognition!");
    }


    // Create the entities from the bounding boxes
    people_msgs::PositionMeasurementArray people;
    sara_msgs::Entities Entities;


    int i{0};
    for (auto &boundingBox : BoundingBoxes3D) {
        sara_msgs::Entity en;
        en.BoundingBox = boundingBox;
        en.name = boundingBox.Class;
        en.position = boundingBox.Center;
        en.probability = boundingBox.probability;
        if (Colors.size() == BoundingBoxes3D.size())
            en.color = Colors[i].color;

        sara_msgs::Entity_<std::allocator<void>>::_name_type t;
        t = en.name;
        if (en.name == "person") {
            people_msgs::PositionMeasurement person;
            en.position.z = 0;

            // Compare the legs with their people
            for (auto &legs : Legs.people) {
                if (legs.reliability && sqrt(
                        (legs.pos.x - en.position.x) * (legs.pos.x - en.position.x) +
                        (legs.pos.y - en.position.y) * (legs.pos.y - en.position.y)) < _LEG_DISTANCE) {
                    en.position = legs.pos;
                    legs.reliability = 0;
                }
            }

            person.pos = en.position;
            person.reliability = en.probability;

            if (person.reliability)
                people.people.push_back(person);
        }
        if (en.probability)
            Entities.entities.push_back(en);
        ++i;
    }

    for (auto legs : Legs.people) {
        if (legs.reliability) {
            sara_msgs::Entity en;
            en.position = legs.pos;
            en.probability = legs.reliability/2;
            Entities.entities.push_back(en);
        }
    }


    //TODO:3Pose Call
    //TODO:Face Recognition Call
    //TODO:Gender Recognition Call


    ROS_INFO("publishing entities");
    entityPublisher.publish(Entities);
    peoplePublisher.publish(people);
}


/**
 * Receive a list of bounding boxes from Darknet and convert them to sara standard bounding boxes
 * @param DBBs 		Darknet bounding boxes
 * @return BBsOut 		sara bounding boxes
 */
std::vector<sara_msgs::BoundingBox2D> ConvertBB(std::vector<darknet_ros_msgs::BoundingBox> DBBs) {
    std::vector<sara_msgs::BoundingBox2D> BBsOut;
    for (auto DBB : DBBs) {
        sara_msgs::BoundingBox2D BBOut;
        BBOut.Class = DBB.Class;
        BBOut.probability = DBB.probability;
        BBOut.xmax = DBB.xmax;
        BBOut.xmin = DBB.xmin;
        BBOut.ymax = DBB.ymax;
        BBOut.ymin = DBB.ymin;
        BBsOut.push_back(BBOut);
    }
    return BBsOut;
}


int main(int argc, char **argv) {

    DataCollector Collector(argc, argv);

}