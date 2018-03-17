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
std::string _MARKER_TOPIC;
double _LEG_DISTANCE;


DataCollector::DataCollector(int argc, char **argv) {
    // Initialise ROS
    ros::init(argc, argv, "wm_data_collector");
    ros::NodeHandle nh;

    // Get all parameters
    nh.param("camera_topic", _CAMERA_TOPIC, std::string("/head_xtion/rgb/image_raw"));
    ROS_INFO("camera_topic = %s", _CAMERA_TOPIC.c_str());
    nh.param("depth_camera_topic", _DEPTH_CAMERA_TOPIC, std::string("/head_xtion/depth/image_raw"));
    ROS_INFO("depth_camera_topic = %s", _DEPTH_CAMERA_TOPIC.c_str());
    nh.param("yolo_topic", _YOLO_TOPIC, std::string("/darknet_ros/bounding_boxes"));
    ROS_INFO("yolo_topic = %s", _YOLO_TOPIC.c_str());
    nh.param("entities_topic", _ENTITIES, std::string("/entities"));
    ROS_INFO("entities_topic = %s", _ENTITIES.c_str());
    nh.param("people_topic", _PEOPLE_TOPIC, std::string("/people"));
    ROS_INFO("people_topic = %s", _PEOPLE_TOPIC.c_str());
    nh.param("legs_topic", _LEGS_TOPIC, std::string("/legs"));
    ROS_INFO("legs_topic = %s", _LEGS_TOPIC.c_str());
    nh.param("entities_marker_topic", _MARKER_TOPIC, std::string("/entities_marker"));
    ROS_INFO("entities_marker_topic = %s", _MARKER_TOPIC.c_str());
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
    entityPublisher = nh.advertise<sara_msgs::Entities>(_ENTITIES, 100);
    peoplePublisher = nh.advertise<people_msgs::PositionMeasurement>(_PEOPLE_TOPIC, 100);
    markerPublisher = nh.advertise<visualization_msgs::Marker>(_MARKER_TOPIC, 100);

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


void DataCollector::PublishVisualisation() {
    // Publish the visualisation markers for rviz
    int i{0};
    for (auto &en : Entities.entities) {
        // Publish position dot

        if (en.name != "") {
            {
                visualization_msgs::Marker m;
                m.header.stamp = ros::Time::now();
                m.lifetime = ros::Duration(0.5);
                m.header.frame_id = "/map";
                m.ns = "entities";
                m.id = i++;
                m.type = m.CYLINDER;
                m.pose.position = en.position;
                m.scale.x = 0.3;
                m.scale.y = 0.3;
                m.scale.z = 0.3;
                m.color.r = float(1 - en.probability);
                m.color.g = float(en.probability);
                m.color.b = 0.1;
                m.color.a = 1;
                markerPublisher.publish(m);
            }

            // Publish name on top
            {
                visualization_msgs::Marker m;
                m.header.stamp = ros::Time::now();
                m.lifetime = ros::Duration(0.5);
                m.header.frame_id = "/map";
                m.ns = "entities";
                m.id = i++;
                m.type = m.TEXT_VIEW_FACING;
                m.text = en.name;
                m.pose.position.x = en.position.x;
                m.pose.position.y = en.position.y;
                m.pose.position.z = en.position.z + 0.5;
                m.scale.x = 0.2;
                m.scale.y = 0.2;
                m.scale.z = 0.2;
                m.color.r = 1;
                m.color.g = 1;
                m.color.b = 1;
                m.color.a = 1;
                markerPublisher.publish(m);
            }
        }
    }
}



/**
 * Receive a list of bounding boxes and create entities from them
 * @param msg 		The ros message
 */
void DataCollector::BoundingBoxCallback(darknet_ros_msgs::BoundingBoxes msg) {

    if (!Image || !DepthImage)
        return;

//    ROS_INFO("BB received");
    // Convert from darknet format to sara format
    auto BoundingBoxes2D = ConvertBB(msg.boundingBoxes);


    // Convert the 2D bounding boxes into 3D ones
    wm_frame_to_box::GetBoundingBoxes3D BBService;
    BBService.request.boundingBoxes2D = BoundingBoxes2D;
    BBService.request.image = *DepthImage;
    BBService.request.output_frame = "/map";
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

    {  // Remove too old entities
        int i{0};
        for (auto &en : Entities.entities) {
            en.probability /= 1.1;
            if (en.probability < 0.1) {
                Entities.entities.erase(Entities.entities.begin()+i);
                i--;
            }
            i++;
        }
    }

    int i{0};
    for (auto &boundingBox : BoundingBoxes3D) {


        sara_msgs::Entity en;
        en.BoundingBox = boundingBox;
        en.name = boundingBox.Class;
        en.position = boundingBox.Center;
        en.probability = boundingBox.probability;
        en.lastUpdateTime = ros::Time::now();
        if (Colors.size() == BoundingBoxes3D.size())
            en.color = Colors[i].color;

        // If the entity is a person
        if (en.name == "person") {
            people_msgs::PositionMeasurement person;
            en.position.z = 0;

            // we compare it with the list of legs
            double minDistance{_LEG_DISTANCE};
            auto *closestLegs{&Legs.people[0]};
            // Compare the legs with their people
            for (auto &legs : Legs.people) {
                if (legs.reliability > 0) {
                    double distance = sqrt(
                            (legs.pos.x - en.position.x+en.velocity.x) * (legs.pos.x - en.position.x+en.velocity.x) +
                            (legs.pos.y - en.position.y+en.velocity.y) * (legs.pos.y - en.position.y+en.velocity.y));
                    if (distance < minDistance) {
                        ROS_INFO("matching legs with person");
                        closestLegs = &legs;
                        minDistance = distance;
                    }
                }
            }
            en.position = closestLegs->pos;
            closestLegs->reliability = 0;

            // We publish a seed for leg detector
            person.pos.x = en.position.x;
            person.pos.y = en.position.y;
            person.object_id = en.ID;
            person.initialization = 1;
            person.name = en.name;
            person.reliability = en.probability;
            peoplePublisher.publish(person);
        }


        // Looking for the closest entity from the past
        if (en.probability > 0) {
            sara_msgs::Entity *closestEntity{nullptr};
            double minDist{2};
            for (auto &en2 : Entities.entities){
                // Calculate the weighted resemblances
                double distance{ sqrt((en.position.x+en.velocity.x-en2.position.x-en2.velocity.x)*(en.position.x+en.velocity.x-en2.position.x-en2.velocity.x) +
                                (en.position.y+en.velocity.y-en2.position.y-en2.velocity.y)*(en.position.y+en.velocity.y-en2.position.y-en2.velocity.y) +
                                (en.position.z+en.velocity.z-en2.position.z-en2.velocity.z)*(en.position.z+en.velocity.z-en2.position.z-en2.velocity.z))};
                distance += (en.name == en2.name)*0.5;
                distance += (en.color == en2.color)*0.1;
                distance += (en.gender == en2.gender)*0.1;
                if (distance < minDist){
                    closestEntity = &en2;
                    minDist = distance;
                }
            }
            if (closestEntity != nullptr){
                ROS_INFO("matching with old entity");
                closestEntity->name = en.name;
                closestEntity->position = en.position;
                closestEntity->color = en.color;
                closestEntity->BoundingBox = en.BoundingBox;
                closestEntity->emotion = en.emotion;
                closestEntity->direction = en.direction;
                closestEntity->probability = en.probability;
                closestEntity->lastUpdateTime = en.lastUpdateTime;
                closestEntity->velocity.x = (en.position.x-closestEntity->position.x)/2;
                closestEntity->velocity.x = (en.position.y-closestEntity->position.y)/2;
                closestEntity->velocity.x = (en.position.z-closestEntity->position.z)/2;
            } else {
                Entities.entities.push_back(en);
            }



        } else {
            ROS_INFO("rejecting an entity of name : %s because it's probability is %lf", en.name.c_str(), en.probability);
        }
        ++i;
    }

    for (auto legs : Legs.people) {
        if (legs.reliability > 0) {
            sara_msgs::Entity en;
            en.position = legs.pos;
            en.probability = legs.reliability / 2;


            sara_msgs::Entity *closestEntity{nullptr};
            double minDist{0.5};
            for (auto &en2 : Entities.entities){
                // Calculate the weighted resemblances
                double distance{ sqrt((en.position.x+en.velocity.x-en2.position.x-en2.velocity.x)*(en.position.x+en.velocity.x-en2.position.x-en2.velocity.x) +
                (en.position.y+en.velocity.y-en2.position.y-en2.velocity.y)*(en.position.y+en.velocity.y-en2.position.y-en2.velocity.y) +
                (en.position.z+en.velocity.z-en2.position.z-en2.velocity.z)*(en.position.z+en.velocity.z-en2.position.z-en2.velocity.z))};

            if (distance < minDist){
                    closestEntity = &en2;
                    minDist = distance;
                }
            }
            if (closestEntity != nullptr){
                ROS_INFO("matching with old entity");
                closestEntity->position = en.position;
                closestEntity->direction = en.direction;
                closestEntity->lastUpdateTime = en.lastUpdateTime;
                closestEntity->probability = en.probability;
                closestEntity->velocity.x = (en.position.x-closestEntity->position.x)/2;
                closestEntity->velocity.x = (en.position.y-closestEntity->position.y)/2;
                closestEntity->velocity.x = (en.position.z-closestEntity->position.z)/2;
            } else {
                Entities.entities.push_back(en);
            }

        }
    }
















    //TODO:Face Recognition Call
    //TODO:Gender Recognition Call



    PublishVisualisation();

    ROS_INFO("publishing %d entities", Entities.entities.size());
    entityPublisher.publish(Entities);
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