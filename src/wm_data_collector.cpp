//
// Created by lucas and Philippe on 21/01/18.
//

#include "wm_data_collector.h"
#include <limits>
#include "ColorComparator.h"

/**
 * Main call. Create the DataCollector object
 */
int main(int argc, char **argv) {

    ros::init(argc, argv, "wm_data_collector");
    ros::NodeHandle nodeHandle;
    ROS_INFO("Starting data colector node");
    DataCollector Collector(nodeHandle);

}


/**
 * DataCollector constructor
 */
DataCollector::DataCollector(ros::NodeHandle nh)
        : nodeHandle_(nh)
        , imageTransport_(nodeHandle_)
        , MyFaceAssignator()
{

    // Get all parameters
    nh.param("subscribers/camera_topic", _CAMERA_TOPIC, std::string("/head_xtion/rgb/image_raw"));
    nh.param("subscribers/depth_camera_topic", _DEPTH_CAMERA_TOPIC, std::string("/head_xtion/depth/image_raw"));

    nh.param("subscribers/faces", _FACES, std::string("/SaraFaceDetector/face"));

    nh.param("subscribers/yolo_topic", _YOLO_TOPIC, std::string("/darknet_ros/bounding_boxes"));
    nh.param("subscribers/entities_topic", _ENTITIES, std::string("/entities"));
    nh.param("subscribers/people_topic", _PEOPLE_TOPIC, std::string("/people"));
    nh.param("subscribers/legs_topic", _LEGS_TOPIC, std::string("/legs"));
    nh.param("subscribers/entities_marker_topic", _MARKER_TOPIC, std::string("/entities_marker"));

    // Entity kinematic parameters
    nh.param("entities/decay", _ENTITY_DECAY, 0.01);
    nh.param("entities/friction", _ENTITY_FRICTION, 0.8);
    nh.param("entities/publication_threshold", _THRESHOLD, 0.5);
    nh.param("entities/max_probability", _MAX_PROBABILITY, 1.0);

    // post marge parameters
    nh.param("post_merge/object/tolerence", _POST_MERGE_TOLERENCE, 2.0);
    nh.param("post_merge/object/max_distance", _POST_MERGE_MAX_DISTANCE, 0.116);
    nh.param("post_merge/object/speed_ratio", _POST_MERGE_SPEED_RATIO, 10.0);
    nh.param("post_merge/people/tolerence_ratio", _POST_MERGE_PEOPLE_TOLERENCE_RATIO, 2.0);
    nh.param("post_merge/people/max_distance", _POST_MERGE_PEOPLE_MAX_DISTANCE, 0.116);
    nh.param("post_merge/people/speed_ratio", _POST_MERGE_PEOPLE_SPEED_RATIO, 10.0);

    // leg parameters
    nh.param("leg_merge/tolerance", _LEG_MERGE_TOLERANCE, 0.6);
    nh.param("leg_merge/max_distance", _LEG_MERGE_MAX_DISTANCE, 0.6);
    nh.param("leg_merge/cumulation", _LEG_MERGE_CUMULATION, 0.6);
    nh.param("leg_merge/speed_ratio", _LEG_MERGE_SPEED_RATIO, 10.0);

    // Camera parameters
    nh.param("camera_merge/object/tolerance", _CAMERA_MERGE_TOLERANCE, 0.8);
    nh.param("camera_merge/object/max_distance", _CAMERA_MERGE_MAX_DISTANCE, 0.8);
    nh.param("camera_merge/object/cumulation", _CAMERA_MERGE_CUMULATION, 0.1);
    nh.param("camera_merge/object/speed_ratio", _CAMERA_MERGE_SPEED_RATIO, 10.0);
    nh.param("camera_merge/people/tolerance", _CAMERA_MERGE_PEOPLE_MERGE_TOLERANCE, 0.8);
    nh.param("camera_merge/people/max_distance", _CAMERA_MERGE_PEOPLE_MAX_DISTANCE, 0.8);
    nh.param("camera_merge/people/cumulation", _CAMERA_MERGE_PEOPLE_CUMULATION, 0.1);
    nh.param("camera_merge/people/speed_ratio", _CAMERA_MERGE_PEOPLE_SPEED_RATIO, 10.0);
    nh.param("camera_merge/face/tolerance", _CAMERA_MERGE_FACE_MERGE_TOLERANCE, 0.8);
    nh.param("camera_merge/face/max_distance", _CAMERA_MERGE_FACE_MAX_DISTANCE, 0.8);
    nh.param("camera_merge/face/cumulation", _CAMERA_MERGE_FACE_CUMULATION, 0.1);
    nh.param("camera_merge/face/speed_ratio", _CAMERA_MERGE_FACE_SPEED_RATIO, 10.0);

    // Weights parameters
    nh.param("weights/name", _NAME_WEIGHT, 1.0);
    nh.param("weights/color", _COLOR_WEIGHT, 0.6);
    nh.param("weights/gender", _GENDER_WEIGHT, 0.6);
    nh.param("weights/position", _POSITION_WEIGHT, 0.1);
    nh.param("weights/face", _FACE_WEIGHT, 10.0);

    ROS_INFO("subscribing to camera topics");
    // Subscribers
    imageSubscriber_ = imageTransport_.subscribe(_CAMERA_TOPIC, 1, &DataCollector::ImageCallback, this);
    imageDepthSubscriber_ = imageTransport_.subscribe(_DEPTH_CAMERA_TOPIC, 1, &DataCollector::DepthImageCallback, this);

    ros::Subscriber yolo_sub = nh.subscribe(_YOLO_TOPIC, 1, &DataCollector::BoundingBoxCallback, this);
    ros::Subscriber leg_sub = nh.subscribe(_LEGS_TOPIC, 1, &DataCollector::LegsCallback, this);
    ros::Subscriber faces_sub = nh.subscribe(_FACES, 1, &DataCollector::FacesCallback, this);

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
    ros::Rate rate(10.0); // run at 20 hz
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

//    MyFaceAssignator.PrintFaceAssignations();

    // Decay entities and remove the old ones
    for (auto &en : Entities.entities)
    {
        en.probability -= _ENTITY_DECAY;
        if(en.probability<0)
            en.probability = 0;
    }


    // Limit the probability level
    for (auto &en : Entities.entities)
        en.probability = en.probability > _MAX_PROBABILITY ? _MAX_PROBABILITY : en.probability;

    // Merge the entities that are too similar to each others
    for (auto en1 : Entities.entities) {
        sara_msgs::Entity *closestEntity{nullptr};
        double minDiff{_POST_MERGE_TOLERENCE};
        for (auto &en2 : Entities.entities) {
            if (en1.ID < en2.ID) {

                double Difference{0};
                if (en1.name == "person" && en2.name == "person")
                    if( ( (en1.face.id.empty()
                           && !en2.face.id.empty()
                           && ros::Time::now().toSec() - MyFaceAssignator.GetFaceLastUpdateTime(en2.face.id).toSec() < 2))
                        || ( (!en1.face.id.empty()
                              && en2.face.id.empty()
                              && ros::Time::now().toSec() - MyFaceAssignator.GetFaceLastUpdateTime(en1.face.id).toSec() < 2) ) )
                    {
                        Difference = CompareEntities(en1, en2, _POST_MERGE_PEOPLE_MAX_DISTANCE)*_POST_MERGE_PEOPLE_TOLERENCE_RATIO;
                    }
                    else
                    {
                        Difference = DBL_MAX;
                    }

                else
                    Difference = CompareEntities(en1, en2, _POST_MERGE_MAX_DISTANCE);

                if (Difference < minDiff ) {
                    closestEntity = &en2;
                    minDiff = Difference;
                }
            }
        }
        if (closestEntity != nullptr && closestEntity->probability>_THRESHOLD){

            ROS_INFO("Merging existing entities");
            MergeEntities(en1, *closestEntity, _POST_MERGE_SPEED_RATIO);
            closestEntity->probability = 0;

        }
    }


    int i{0};
    for (auto &en : Entities.entities) {
        if (en.probability <= 0 && en.face.id.empty() ) {
            ROS_INFO("removing %s %d from list", en.name.c_str(), en.ID);
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

    entityPublisher.publish(Publication);
}


/**
 * Add an entity to the list or match it with a corresponding one
 * @param newEntity 		The entity to add
 * @param tolerance 		the level of tolerance for the match
 */
void DataCollector::AddEntity(sara_msgs::Entity newEntity, double tolerance, double MaxDistance, double ratio) {

    sara_msgs::Entity *closestEntity{nullptr};
    double minDiff{tolerance};
    // Calculate the weighted resemblances and return the most resemblant one
    for (auto &en2 : Entities.entities){
        double Difference = CompareEntities(newEntity, en2, MaxDistance);
        if (Difference < minDiff){
            closestEntity = &en2;
            minDiff = Difference;
            if(newEntity.face.id == en2.face.id)
            {
                MergeEntities(en2, newEntity, _CAMERA_MERGE_FACE_SPEED_RATIO);
                return;
            }
        }
    }

    // If the difference is tolerated, we merge the entities
    if (closestEntity != nullptr){
        MergeEntities(*closestEntity, newEntity, ratio);
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

    // Check if the entities are from the same "frame".
    if (en1.lastUpdateTime == en2.lastUpdateTime && (en1.name != "leg" || en2.name != "leg"))
        return DBL_MAX;

    // Get the distance between the entities
    double Difference{ sqrt((en1.position.x-en2.position.x)*(en1.position.x-en2.position.x) +
                            (en1.position.y-en2.position.y)*(en1.position.y-en2.position.y) +
                            (en1.position.z-en2.position.z)*(en1.position.z-en2.position.z))};


    // If the distance is furter than the max, we return an infinite value
    if (Difference > MaxDistance ) return DBL_MAX;
    Difference *= _POSITION_WEIGHT;

    // Consider the face of the entities
    if ( !en1.face.id.empty() && !en2.face.id.empty() ) //both have a face
    {
        if(en1.face.id == en2.face.id && Difference < 0.20)
            return 0;
        else
            Difference += _FACE_WEIGHT;
    }

    // If the entities are legs or persons, match them more
    if (!en1.name.empty() && !en2.name.empty())
        if (!(en1.name == "person" && en2.name == "legs" || en1.name == "legs" && en2.name == "person" ))
            Difference += double(en1.name != en2.name)*_NAME_WEIGHT;

    // Consider the color of the entities
    if ( !(en1.name == "person" || en2.name == "person") && !en1.color.empty() && !en2.color.empty())
        Difference += (1-ColorComparison::CompareColors(en1.color, en2.color))*_COLOR_WEIGHT;
//
//    // Consider the gender of the entities
    Difference += double(!en1.face.id.empty() && !en2.face.id.empty() && en1.face.gender != en2.face.gender)*_GENDER_WEIGHT;



    return Difference;
}

/**
 * Merge two entities into one and return the result
 * @param Target 		        The target entity on which to merge the source
 * @param Source 		        The source entity to merge on the target
 */
void DataCollector::MergeEntities(sara_msgs::Entity &Target, sara_msgs::Entity &Source, double ratio) {


    Target.probability += Source.probability;

    Target.ID = Target.ID==0 ? Source.ID : Target.ID;

    // Manage special case of persons and legs
    if (Target.name == "legs" && Source.name == "person" || Source.name == "legs" && Target.name == "person")
        Target.name = "person";
    else if (Target.name == "face" && Source.name == "person" || Source.name == "face" && Target.name == "person"){
        Target.name = "person";
    } else
        Target.name = Target.name.empty() ? Source.name : Target.name;

    // Merge properties

    if (Target.color.empty() || !Target.color.empty() && !Source.color.empty())
        Target.color = Source.color;
//    if (Target.gender.empty() || !Target.gender.empty() && !Source.gender.empty())
//        Target.gender = Source.gender;
//    if (Target.emotion.empty() || !Target.emotion.empty() && !Source.emotion.empty())
//        Target.emotion = Source.emotion;
    if (!Target.BoundingBox.probability || Target.probability && Source.probability)
        Target.BoundingBox = Source.BoundingBox;

    if (Source.face.id.empty())
        Source.face = Target.face;

    //Merge face on person
    if (!Source.face.id.empty())
    {
        Target.face = Source.face;
        Source.face.id = "";
    }

    if(!Source.face.id.empty() && !Target.face.id.empty())
    {
        if(Source.face.id != Target.face.id)
        {
            MyFaceAssignator.MergeAssignation(Target.ID, Source.ID, Source.face.id);
        }
    }


    Target.lastUpdateTime = Source.lastUpdateTime;
    Target.aliases.insert(Target.aliases.end(), Source.aliases.begin(), Source.aliases.end());

    double dx{Source.position.x-Target.position.x};
    double dy{Source.position.y-Target.position.y};
    double dz{Source.position.z-Target.position.z};
    double dist{sqrt(pow(dx,2)+pow(dy,2)+pow(dz,2))};

    double min{sqrt(pow(Target.BoundingBox.Depth,2)+pow(Target.BoundingBox.Height,2)+pow(Target.BoundingBox.Width,2))/2};
    if (min < 0.2) min = 0.2;

    if (dist < min) dist = min;

    Target.velocity.x += dx / dist * ratio;
    Target.velocity.y += dy / dist * ratio;
    Target.velocity.z += dz / dist * ratio;
    Target.position.x += (Source.position.x-Target.position.x)*ratio;
    Target.position.y += (Source.position.y-Target.position.y)*ratio;
    Target.position.z += (Source.position.z-Target.position.z)*ratio;


    Target.probability += Source.probability;

    if (Target.probability > _MAX_PROBABILITY) Target.probability = _MAX_PROBABILITY;

    int i{0};
    {  // Publish the entity core input
        visualization_msgs::Marker m;
        m.header.stamp = ros::Time::now();
        m.lifetime = ros::Duration(0.2);
        m.header.frame_id = "/map";
        m.ns = "entities_merge";
        m.id = ros::Time::now().toNSec();
        m.type = m.SPHERE;
        m.pose.position.x = Source.position.x;
        m.pose.position.y = Source.position.y;
        m.pose.position.z = Source.position.z;
        m.scale.x = 0.05;
        m.scale.y = 0.05;
        m.scale.z = 0.05;
        m.color.r = 1;
        m.color.g = 0;
        m.color.b = 0;
        m.color.a = 1;
        markerPublisher.publish(m);
    }
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
                m.type = m.SPHERE;
                m.pose.position = en.position;
                m.scale.x = 0.15;
                m.scale.y = 0.15;
                m.scale.z = 0.15;
                m.color.r = float(1 - en.probability);
                m.color.g = float(en.probability);
                m.color.b = 0.1;
                m.color.a = 0.5;
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
                temp = en.color + " " + en.name;
                m.text = temp;
                m.pose.position.x = en.position.x;
                m.pose.position.y = en.position.y;
                m.pose.position.z = en.position.z + 0.2;
                m.scale.x = 0.08;
                m.scale.y = 0.08;
                m.scale.z = 0.08;
                m.color.r = 1;
                m.color.g = 1;
                m.color.b = 0.0;
                m.color.a = 1;
                markerPublisher.publish(m);
            }
//            {  // Publish probability on top
//                visualization_msgs::Marker m;
//                m.header.stamp = ros::Time::now();
//                m.lifetime = ros::Duration(0.5);
//                m.header.frame_id = "/map";
//                m.ns = "entities";
//                m.id = i++;
//                m.type = m.TEXT_VIEW_FACING;
//                std::string temp;
//                temp = std::to_string(en.probability);
//                m.text = temp;
//                m.pose.position.x = en.position.x;
//                m.pose.position.y = en.position.y;
//                m.pose.position.z = en.position.z + 0.4;
//                m.scale.x = 0.1;
//                m.scale.y = 0.1;
//                m.scale.z = 0.1;
//                m.color.r = 1;
//                m.color.g = 1;
//                m.color.b = 1;
//                m.color.a = 1;
//                markerPublisher.publish(m);
//            }
            {  // Publish ID on under it
                visualization_msgs::Marker m;
                m.header.stamp = ros::Time::now();
                m.lifetime = ros::Duration(0.5);
                m.header.frame_id = "/map";
                m.ns = "entities";
                m.id = i++;
                m.type = m.TEXT_VIEW_FACING;
                std::string temp;
                temp = std::to_string(en.ID);
                m.text = temp;
                m.pose.position.x = en.position.x;
                m.pose.position.y = en.position.y;
                m.pose.position.z = en.position.z + 0.10;
                m.scale.x = 0.12;
                m.scale.y = 0.12;
                m.scale.z = 0.12;
                m.color.r = 1;
                m.color.g = 1;
                m.color.b = 0.0;
                m.color.a = 1;
                markerPublisher.publish(m);
            }
            if (en.face.id != ""){  // Publish faces on over it
                visualization_msgs::Marker m;
                m.header.stamp = ros::Time::now();
                m.lifetime = ros::Duration(0.5);
                m.header.frame_id = "/map";
                m.ns = "faceMarker";
                m.id = i++;
                m.type = m.SPHERE;
                m.pose.position = en.position;
                m.pose.position.z += 0.1;
                m.scale.x = 0.1;
                m.scale.y = 0.1;
                m.scale.z = 0.1;
                m.color.r = 0;
                m.color.g = 0;
                m.color.b = 0;
                m.color.a = 0.5;
                markerPublisher.publish(m);

                {  // Publish ID on under it
                    visualization_msgs::Marker m;
                    m.header.stamp = ros::Time::now();
                    m.lifetime = ros::Duration(0.5);
                    m.header.frame_id = "/map";
                    m.ns = "faceID";
                    m.id = i++;
                    m.type = m.TEXT_VIEW_FACING;
                    std::string temp;
                    temp = en.face.id;
                    m.text = temp;
                    m.pose.position.x = en.position.x;
                    m.pose.position.y = en.position.y;
                    m.pose.position.z = en.position.z + 0.50;
                    m.scale.x = 0.12;
                    m.scale.y = 0.12;
                    m.scale.z = 0.12;
                    m.color.r = 1;
                    m.color.g = 1;
                    m.color.b = 0.0;
                    m.color.a = 1;
                    markerPublisher.publish(m);
                }

            }
        }
    }
}



/**
 * Receive an image from camera and save it
 * @param msg 		The ros message
 */
void DataCollector::ImageCallback(const sensor_msgs::ImageConstPtr& msg) {
//    ROS_INFO("image received");

    Image = std::move(msg);
    return;
}


/**
 * Receive a depth image from camera and save it
 * @param msg 		The ros message
 */
void DataCollector::DepthImageCallback(const sensor_msgs::ImageConstPtr& msg) {
//    ROS_INFO("image received");
    DepthImage = msg;
    return;
}


/**
 * Receive a list of legs and create entities from them
 * @param Legs 		The list of persons found by their legs
 */
void DataCollector::LegsCallback(people_msgs::PositionMeasurementArray Legs) {
    for (auto legs : Legs.people) {
        if (legs.reliability > 0) {
            //ROS_INFO("leg ID: %d", legs. );
            sara_msgs::Entity en;
            en.position = legs.pos;
            en.probability = _LEG_MERGE_CUMULATION;
            en.name = "legs";
            en.lastUpdateTime = legs.header.stamp;

            sara_msgs::Entity *closestEntity{nullptr};
            double minDiff{_LEG_MERGE_TOLERANCE};
            // Calculate the weighted resemblances and return the most resemblant one
            for (auto &en2 : Entities.entities){
                if (en2.name == "person") {
                    double dx{en2.position.x - en.position.x};
                    double dy{en2.position.y - en.position.y};
                    double dist{sqrt(pow(dx, 2) + pow(dy, 2))};

                    double vel{dist < 0.5 ? 0.5 : dist};
                    if (dist < _LEG_MERGE_MAX_DISTANCE){
                        en2.velocity.x -= dx / vel * _LEG_MERGE_SPEED_RATIO;
                        en2.velocity.y -= dy / vel * _LEG_MERGE_SPEED_RATIO;
                        en2.position.x += (en.position.x-en2.position.x)*_LEG_MERGE_SPEED_RATIO;
                        en2.position.y += (en.position.y-en2.position.y)*_LEG_MERGE_SPEED_RATIO;
                        if (dist < _LEG_MERGE_TOLERANCE)
                            en2.probability += _LEG_MERGE_CUMULATION;
                    }
                }
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

    // Convert from darknet format to sara format
    auto BoundingBoxes2D = ConvertBB(msg);

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
    if (Colors.size() != BoundingBoxes3D.boundingBoxes.size()) {
        ROS_ERROR("An error occurred with Color Recognition! (numbers doesn't match)");
        return;
    }

    // Create the entities from the bounding boxes
    int i{0};
    for (auto &boundingBox : BoundingBoxes3D.boundingBoxes) {

        sara_msgs::Entity en;
        en.BoundingBox = boundingBox;
        en.name = boundingBox.Class;
        en.position = boundingBox.Center;
        en.probability = boundingBox.probability;
        en.lastUpdateTime = BoundingBoxes3D.header.stamp;
        if (Colors.size() == BoundingBoxes3D.boundingBoxes.size())
            en.color = Colors[i].color;
        // Looking for the closest entity from the past
        // TODO: replace by a rosparam.
        if (en.probability > 0.1) {
            // ROS_INFO("adding an entity of name : %s because it's probability is %lf", en.name.c_str(), en.probability);

            // If the entity is a person
            en.probability *= 2;
            if (en.name == "person") {
                en.position.z = 0;
                en.probability = _CAMERA_MERGE_PEOPLE_CUMULATION;
                AddEntity(en, _CAMERA_MERGE_PEOPLE_MERGE_TOLERANCE, _CAMERA_MERGE_PEOPLE_MAX_DISTANCE, _CAMERA_MERGE_PEOPLE_SPEED_RATIO);
            }else{
                en.probability = _CAMERA_MERGE_CUMULATION;
                AddEntity(en, _CAMERA_MERGE_TOLERANCE, _CAMERA_MERGE_MAX_DISTANCE, _CAMERA_MERGE_SPEED_RATIO);
            }
        } else {
            ROS_WARN("rejecting an entity of name : %s because it's probability is %lf", en.name.c_str(), en.probability);
        }
        ++i;
    }
}

/**
 * Receive a list of faces and create entities from them
 * @param msg 		The ros message
 */
void DataCollector::FacesCallback(sara_msgs::Faces msg) {

    // Create the entities from the bounding boxes
    for (auto face : msg.faces) {

        if(face.boundingBox.probability == 0)
        {
            ROS_WARN("Face %s ignored due to a probability of 0", face.id.c_str());
            continue;
        }

        sara_msgs::Entity en;
        en.BoundingBox = face.boundingBox;
        en.name = "face";
        en.position = face.boundingBox.Center;
        en.lastUpdateTime = msg.header.stamp;
        en.position.z = 0;
        en.probability = _CAMERA_MERGE_FACE_CUMULATION;
        en.face = face;

        MyFaceAssignator.UpdateFaceLastTimeSeenVector(en.face.id, en.lastUpdateTime);

        int EntityID = MyFaceAssignator.GetEntityByFace(&(en.face));

        //if face already associated with an entity
        if (EntityID > 0) {
            sara_msgs::Entity *associatedEntity = GetEntityByID(EntityID);

            if(associatedEntity != nullptr)
            {
                if(!associatedEntity->face.id.empty() && (ros::Time::now().toSec() - MyFaceAssignator.GetFaceLastUpdateTime(associatedEntity->face.id).toSec()) < 0.5 )
                    if(((associatedEntity->position.x-en.position.x)*(associatedEntity->position.x-en.position.x)+(associatedEntity->position.y-en.position.y)*(associatedEntity->position.y-en.position.y)) > _CAMERA_MERGE_FACE_MAX_DISTANCE )
                        continue;
                associatedEntity->face = en.face;
                associatedEntity->position = en.position;
                associatedEntity->position.z = 0;
                associatedEntity->lastUpdateTime = en.lastUpdateTime;
                associatedEntity->probability = 1;
                associatedEntity->velocity.x = 0;
                associatedEntity->velocity.y = 0;
                associatedEntity->velocity.z = 0;
            }

            continue;
        }


        sara_msgs::Entity *closestEntity{nullptr};
        double minDiff{_CAMERA_MERGE_FACE_MAX_DISTANCE};
        for (auto &en2 : Entities.entities) {
            double Difference{ sqrt((en.position.x-en2.position.x)*(en.position.x-en2.position.x) +
                                    (en.position.y-en2.position.y)*(en.position.y-en2.position.y) +
                                    (en.position.z-en2.position.z)*(en.position.z-en2.position.z))};

            if (Difference < minDiff && en2.name=="person" && en.lastUpdateTime.nsec != en2.lastUpdateTime.nsec ) {
                closestEntity = &en2;
                minDiff = Difference;
            }

        }

        if (closestEntity != nullptr) {
            if(!closestEntity->face.id.empty() && (ros::Time::now().toSec() - MyFaceAssignator.GetFaceLastUpdateTime(closestEntity->face.id).toSec()) > 2 )
            {
                ROS_INFO("Face %s not assigned to entity %d because face was not seen for more than 2 secondes", en.face.id.c_str(), closestEntity->ID);
            }
            else {
                ROS_INFO("Face %s assigned to entity %d", en.face.id.c_str(), closestEntity->ID);
                closestEntity->face = en.face;
                closestEntity->position = en.position;
                closestEntity->position.z = 0;
                closestEntity->lastUpdateTime = en.lastUpdateTime;
                closestEntity->probability = 1;
                closestEntity->velocity.x = 0;
                closestEntity->velocity.y = 0;
                closestEntity->velocity.z = 0;
                MyFaceAssignator.AddFace(&en.face, closestEntity);
            }
        }
    }
}


sara_msgs::Entity* DataCollector::GetEntityByID( int EntityID )
{
    for(int i=0; i<Entities.entities.size(); i++)
    {
        if(Entities.entities[i].ID == EntityID)
            return &Entities.entities[i];
    }
    return nullptr;
}


/**
 * Receive a list of bounding boxes from Darknet and convert them to sara standard bounding boxes
 * @param DBBs 		Darknet bounding boxes
 * @return BBsOut 		sara bounding boxes
 */
sara_msgs::BoundingBoxes2D ConvertBB(darknet_ros_msgs::BoundingBoxes DBBs) {
    sara_msgs::BoundingBoxes2D BBsOut;
    for (auto DBB : DBBs.boundingBoxes) {
        sara_msgs::BoundingBox2D BBOut;
        BBOut.Class = DBB.Class;
        BBOut.probability = DBB.probability;
        BBOut.xmax = DBB.xmax;
        BBOut.xmin = DBB.xmin;
        BBOut.ymax = DBB.ymax;
        BBOut.ymin = DBB.ymin;
        BBsOut.boundingBoxes.push_back(BBOut);
    }
    BBsOut.header = DBBs.header;
    return BBsOut;
}
