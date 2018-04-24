//
// Created by lucas and Philippe on 21/01/18.
//

#include "wm_data_collector.h"
#include <limits>

/**
 * Main call. Create the DataCollector object
 */
int main(int argc, char **argv) {

    DataCollector Collector(argc, argv);

}


/**
 * DataCollector constructor
 */
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
    nh.param("leg_tolerance", _LEG_TOLERANCE, 0.6);
    ROS_INFO("leg_tolerance = %lf", _LEG_TOLERANCE);
    nh.param("camera_tolerance", _CAMERA_TOLERANCE, 0.8);
    ROS_INFO("camera_tolerance = %lf", _CAMERA_TOLERANCE);
    nh.param("people_tolerance", _PEOPLE_TOLERANCE, 0.8);
    ROS_INFO("people_tolerance = %lf", _PEOPLE_TOLERANCE);
    nh.param("entity_cumulation", _CUMULATION, 0.1);
    ROS_INFO("entity_cumulation = %lf", _CUMULATION);
    nh.param("entity_decay", _ENTITY_DECAY, 0.99);
    ROS_INFO("entity_decay = %lf", _ENTITY_DECAY);
    nh.param("entity_friction", _ENTITY_FRICTION, 0.99);
    ROS_INFO("entity_friction = %lf", _ENTITY_FRICTION);
    nh.param("name_weight", _NAME_WEIGHT, 1.0);
    ROS_INFO("name_weight = %lf", _NAME_WEIGHT);
    nh.param("person_weight", _PERSON_WEIGHT, 1.0);
    ROS_INFO("person_weight = %lf", _PERSON_WEIGHT);
    nh.param("color_weight", _COLOR_WEIGHT, 0.6);
    ROS_INFO("color_weight = %lf", _COLOR_WEIGHT);
    nh.param("gender_weight", _GENDER_WEIGHT, 0.6);
    ROS_INFO("gender_weight = %lf", _GENDER_WEIGHT);
    nh.param("publication_threshold", _THRESHOLD, 0.5);
    ROS_INFO("publication_threshold = %lf", _THRESHOLD);
    nh.param("normalise_probability", _NORMALISE, false);
    ROS_INFO("normalise_probability = %lf", _NORMALISE);
    nh.param("max_probability", _MAX_PROBABILITY, 2.0);
    ROS_INFO("max_probability = %lf", _MAX_PROBABILITY);
    nh.param("speed_ratio", _SPEED_RATIO, 0.1);
    ROS_INFO("speed_ratio = %lf", _SPEED_RATIO);
    nh.param("leg_probability_ratio", _LEG_PROBABILITY_RATIO, 0.116);
    ROS_INFO("leg_probability_ratio = %lf", _LEG_PROBABILITY_RATIO);



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

    // initialyse the procedural ID
    ProceduralID = 1;

    ROS_INFO("running");
    ros::Rate rate(20.0); // run at 20 hz
    while (ros::ok()){
        UpdateEntities();
        PublishVisualisation();
        ros::spinOnce();
        rate.sleep();
    }

}


/**
 * Update all entities end clean the database
 */
void DataCollector::UpdateEntities() {

    // Normalise the probabilities
    if (_NORMALISE) {
        double MaxProb{1};
        for (auto &en : Entities.entities)
            if (en.probability > MaxProb)
                MaxProb = en.probability;
        if (MaxProb > 0)
            for (auto &en : Entities.entities)
                en.probability /= MaxProb;
    }

    // Decay entities and remove the old ones
    for (auto &en : Entities.entities)
        en.probability -= _ENTITY_DECAY;

    // Limit the probability level
    for (auto &en : Entities.entities)
        en.probability = en.probability > _MAX_PROBABILITY ? _MAX_PROBABILITY : en.probability;

    // Merge the entities that are too similar to each others
    for (auto en1 : Entities.entities) {
        sara_msgs::Entity *closestEntity{nullptr};
        double minDiff{1.0};
        for (auto &en2 : Entities.entities) {
            if (en1.ID < en2.ID) {
                double Difference{CompareEntities(en1, en2, 0.2)};
                if (Difference < minDiff) {
                    closestEntity = &en2;
                    minDiff = Difference;
                }
            }
        }
        if (closestEntity != nullptr){
            ROS_INFO("Merging existing entities");
            MergeEntities(en1, *closestEntity);
            closestEntity->probability = 0;
        }
    }


    int i{0};
    for (auto &en : Entities.entities) {
        if (en.probability <= 0) {
            Entities.entities.erase(Entities.entities.begin()+i);
            //ROS_INFO("deleting entity");
            i--;
        }
        i++;
    }

    // Increment velocity to the position and apply virtual friction
    for (auto &en : Entities.entities) {
        en.position.x += en.velocity.x *= _ENTITY_FRICTION;
        en.position.y += en.velocity.y *= _ENTITY_FRICTION;
        en.position.z += en.velocity.z *= _ENTITY_FRICTION;

        // If the entity is a person
        if (en.name == "person") {
            people_msgs::PositionMeasurement person;
            en.position.z = 0;

            // We publish a seed for leg detector
            person.pos.x = en.position.x;
            person.pos.y = en.position.y;
            person.object_id = en.ID;
            person.initialization = 0;
            person.name = en.name;
            person.reliability = en.probability;
            peoplePublisher.publish(person);
        }
    }

    sara_msgs::Entities Publication;
    for (auto &en : Entities.entities)
        if (en.probability > _THRESHOLD)
            Publication.entities.push_back(en);

    entityPublisher.publish(Entities);
}


/**
 * Add an entity to the list or match it with a corresponding one
 * @param newEntity 		The entity to add
 * @param tolerance 		the level of tolerance for the match
 */
void DataCollector::AddEntity(sara_msgs::Entity newEntity, double tolerance) {

    sara_msgs::Entity *closestEntity{nullptr};
    double minDiff{tolerance};
    // Calculate the weighted resemblances and return the most resemblant one
    for (auto &en2 : Entities.entities){
        double Difference = CompareEntities(newEntity, en2, 1);
        if (Difference < minDiff){
            closestEntity = &en2;
            minDiff = Difference;
        }
    }

    // If the difference is tolerated, we merge the entities
    if (closestEntity != nullptr){
        MergeEntities(*closestEntity, newEntity);
    } else {
        // If not, we simply add the entity to the list
        newEntity.ID = ProceduralID++;
        newEntity.probability = 0.2;
        ROS_INFO("Adding new entity: %s : %d", newEntity.name.c_str(), newEntity.ID);
        Entities.entities.push_back(newEntity);
    }
}



/**
 * Compare two entities and return their difference level
 * @param en1 		        The first entity
 * @param en2 		        The second entity
 * @param MaxDistance 		The maximum permited distance between entities
 * @return Difference 		The amount of difference
 */
double DataCollector::CompareEntities(sara_msgs::Entity &en1, sara_msgs::Entity &en2, double MaxDistance=1) {

    // Get the distance between the entities
    double Difference{ sqrt((en1.position.x-en2.position.x)*(en1.position.x-en2.position.x) +
                            (en1.position.y-en2.position.y)*(en1.position.y-en2.position.y) +
                            (en1.position.z-en2.position.z)*(en1.position.z-en2.position.z))};

    // If the distance is furter than the max, we return an infinite value
    if (Difference > MaxDistance) return DBL_MAX;


    //Difference /= 1+en1.probability*en2.probability/10;

    // If the entities are legs or persons, match them more
    if (!en1.name.empty() && !en2.name.empty())
        if (!(en1.name == "person" && en2.name == "legs" || en1.name == "legs" && en2.name == "person" ))
            Difference += double(en1.name != en2.name)*_NAME_WEIGHT;

    // Consider the color of the entities
    Difference += double(!en1.color.empty() && !en2.color.empty() && (en1.color != en2.color)*_COLOR_WEIGHT);

    // Consider the gender of the entities
    Difference += double(!en1.gender.empty() && !en2.gender.empty() & en1.gender != en2.gender)*_GENDER_WEIGHT;

    return Difference;
}

/**
 * Merge two entities into one and return the result
 * @param Target 		        The target entity on which to merge the source
 * @param Source 		        The source entity to merge on the target
 */
void DataCollector::MergeEntities(sara_msgs::Entity &Target, sara_msgs::Entity &Source) {


    Target.probability += Source.probability*_CUMULATION;

    Target.ID = Target.ID==0 ? Source.ID : Target.ID;

    // Manage spacial case of persons and legs
    if (Target.name == "legs" && Source.name == "person" || Source.name == "legs" && Target.name == "person")
        Target.name = "person";
    else
        Target.name = Target.name.empty() ? Source.name : Target.name;

    // Merge properties

    if (Target.color.empty() || !Target.color.empty() && !Source.color.empty() && Source.probability > Target.probability)
        Target.color = Source.color;
    if (Target.gender.empty() || !Target.gender.empty() && !Source.gender.empty() && Source.probability > Target.probability)
        Target.gender = Source.gender;
    if (Target.emotion.empty() || !Target.emotion.empty() && !Source.emotion.empty() && Source.probability > Target.probability)
        Target.emotion = Source.emotion;
    if (!Target.BoundingBox.probability || Target.probability && Source.probability && Source.probability > Target.probability)
        Target.BoundingBox = Source.BoundingBox;
    if (!Target.direction || Target.direction && Source.direction && Source.probability > Target.probability)
        Target.direction = Source.direction;

    Target.lastUpdateTime = ros::Time::now();
    Target.aliases.insert(Target.aliases.end(), Source.aliases.begin(), Source.aliases.end());

    Target.velocity.x += (-Target.position.x+Source.position.x)/_SPEED_RATIO;
    Target.velocity.y += (-Target.position.y+Source.position.y)/_SPEED_RATIO;
    Target.velocity.z += (-Target.position.z+Source.position.z)/_SPEED_RATIO/2;

    Target.probability += Source.probability*_CUMULATION;

    if (Target.probability > _MAX_PROBABILITY) Target.probability = _MAX_PROBABILITY;

}

/**
 * Publish the visual clues onto markers for rviz
 */
void DataCollector::PublishVisualisation() {
    // Publish the visualisation markers for rviz
    int i{0};
    for (auto &en : Entities.entities) {

        if (en.probability > _THRESHOLD) {
            {  // Publish position dot
                visualization_msgs::Marker m;
                m.header.stamp = ros::Time::now();
                m.lifetime = ros::Duration(0.5);
                m.header.frame_id = "/map";
                m.ns = "entities";
                m.id = i++;
                m.type = m.CYLINDER;
                m.pose.position = en.position;
                m.scale.x = 0.2;
                m.scale.y = 0.2;
                m.scale.z = 0.2;
                m.color.r = float(1 - en.probability);
                m.color.g = float(en.probability);
                m.color.b = 0.1;
                m.color.a = 1;
                markerPublisher.publish(m);
            }
            {  // Publish name on top
                visualization_msgs::Marker m;
                m.header.stamp = ros::Time::now();
                m.lifetime = ros::Duration(0.5);
                m.header.frame_id = "/map";
                m.ns = "entities";
                m.id = i++;
                m.type = m.TEXT_VIEW_FACING;
                std::string temp;
                temp = en.color + " " + en.name + ":" + std::to_string(en.ID);
                m.text = temp;
                m.pose.position.x = en.position.x;
                m.pose.position.y = en.position.y;
                m.pose.position.z = en.position.z + 0.5;
                m.scale.x = 0.15;
                m.scale.y = 0.15;
                m.scale.z = 0.15;
                m.color.r = 1;
                m.color.g = 1;
                m.color.b = 1;
                m.color.a = 1;
                markerPublisher.publish(m);
            }
            {  // Publish name on top
                visualization_msgs::Marker m;
                m.header.stamp = ros::Time::now();
                m.lifetime = ros::Duration(0.5);
                m.header.frame_id = "/map";
                m.ns = "entities";
                m.id = i++;
                m.type = m.TEXT_VIEW_FACING;
                std::string temp;
                temp = std::to_string(en.probability);
                m.text = temp;
                m.pose.position.x = en.position.x;
                m.pose.position.y = en.position.y;
                m.pose.position.z = en.position.z + 0.4;
                m.scale.x = 0.1;
                m.scale.y = 0.1;
                m.scale.z = 0.1;
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
 * Receive an image from camera and save it
 * @param msg 		The ros message
 */
void DataCollector::ImageCallback(sensor_msgs::ImageConstPtr msg) {
//    ROS_INFO("image received");
    Image = std::move(msg);
}


/**
 * Receive a depth image from camera and save it
 * @param msg 		The ros message
 */
void DataCollector::DepthImageCallback(sensor_msgs::ImageConstPtr msg) {
//    ROS_INFO("image received");
    DepthImage = std::move(msg);
}


/**
 * Receive a list of legs and create entities from them
 * @param Legs 		The list of persons found by their legs
 */
void DataCollector::LegsCallback(people_msgs::PositionMeasurementArray Legs) {
    for (auto legs : Legs.people) {
        if (legs.reliability > 0) {
            sara_msgs::Entity en;
            en.position = legs.pos;
            en.probability = legs.reliability*_LEG_PROBABILITY_RATIO;
            en.name = "legs";
            AddEntity(en, _LEG_TOLERANCE);
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

    // Check if the number of elements matches
    if (Colors.size() != BoundingBoxes3D.size()) {
        ROS_ERROR("An error occurred with Color Recognition! (numbers doesn't match)");
        return;
    }

    // Create the entities from the bounding boxes
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
        // Looking for the closest entity from the past
        if (en.probability > 0.5) {
            // ROS_INFO("adding an entity of name : %s because it's probability is %lf", en.name.c_str(), en.probability);

            // If the entity is a person
            en.probability *= 2;
            if (en.name == "person") {
                en.position.z = 0;
                AddEntity(en, _CAMERA_TOLERANCE);
            }else{
                AddEntity(en, _PEOPLE_TOLERANCE);
            }
        } else {
            ROS_WARN("rejecting an entity of name : %s because it's probability is %lf", en.name.c_str(), en.probability);
        }
        ++i;
    }
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
