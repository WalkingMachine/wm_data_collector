//
// Created by walking on 5/17/18.
//

#include "FaceAssignator.h"

using namespace std;

FaceAssignator::FaceAssignator() : FaceAssignations(){

}

void FaceAssignator::AddFace( sara_msgs::FaceMsg* Face, sara_msgs::Entity* Entity ){

    FaceAssignation newFace;
    newFace.FaceID = Face->id;
    newFace.EntityID = Entity->ID;

    FaceAssignations.push_back(newFace);

    cout<<Face->id<<" et "<<Entity->name<<" ajoutée a assignator"<<endl;

}

int FaceAssignator::GetEntityByFace( sara_msgs::FaceMsg* newFace ){
    for (auto& Assignation : FaceAssignations){
        if (newFace->id == Assignation.FaceID)
            return Assignation.EntityID;
    }
    return -1;
}

void FaceAssignator::Merge( sara_msgs::Entity* newEntity ){
    for (auto& Assignation : FaceAssignations){
        if (newEntity->face.id == Assignation.FaceID)
            return;
    }
    FaceAssignation newFace;
    newFace.FaceID = newEntity->face.id;
    newFace.EntityID = newEntity->ID;
    FaceAssignations.push_back(newFace);
}

void FaceAssignator::MergeAssignation( int targetEntityID, int sourceEntityID, string sourceFaceID ){
    for (auto& Assignation : FaceAssignations) {
        if (sourceFaceID == Assignation.FaceID && sourceEntityID == Assignation.EntityID) {
            Assignation.EntityID = targetEntityID;
        }
    }
}

void FaceAssignator::PrintFaceAssignations(){
    for (auto &vector : FaceAssignations){
        cout<<"DEBUT PRINT "<<endl;
        cout<<"FaceID "<<vector.FaceID<<endl;
        cout<<"EntityID "<<vector.EntityID<<endl;
        cout<<"FIN PRINT "<<endl;
    }
}