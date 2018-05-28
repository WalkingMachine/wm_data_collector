//
// Created by walking on 5/17/18.
//

#ifndef PROJECT_FACEASSIGNATOR_H
#define PROJECT_FACEASSIGNATOR_H

#include <string>
#include "sara_msgs/Entity.h"

using namespace std;

class FaceAssignator{
    struct FaceAssignation {
        string FaceID;
        int EntityID;
    };
    struct FaceLastTimeSeen {
        string FaceID;
        ros::Time LastSeen;
    };
    vector<FaceAssignation> FaceAssignations;
    vector<FaceLastTimeSeen> FaceLastTimeSeenVector;

public:
    FaceAssignator();
    void AddFace( sara_msgs::FaceMsg* Face, sara_msgs::Entity* Entity );
    int GetEntityByFace( sara_msgs::FaceMsg* Face );
    void Merge( sara_msgs::Entity* newEntity );
    void MergeAssignation( int targetEntityID, int sourceEntityID, string sourceFaceID );
    void PrintFaceAssignations();
    void UpdateFaceLastTimeSeenVector(string FaceId, ros::Time time);
    ros::Time GetFaceLastUpdateTime(string FaceId);
};



#endif //PROJECT_FACEASSIGNATOR_H