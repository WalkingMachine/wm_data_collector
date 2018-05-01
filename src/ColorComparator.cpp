//
// Created by Philippe on 4/30/18.
//

#include "ColorComparator.h"

float ColorComparison::CompareColors(std::string Color1, std::string Color2){
    int ID1, ID2;
    if ( (ID1 = GetColorID(std::move(Color1))) != -1 && (ID2 = GetColorID(std::move(Color2))) != -1 )
        return ColorProximityMatrix[ID1*NBCOLORS+ID2];
    return 0;
}

int ColorComparison::GetColorID(std::string Color){
    if( Color == "red")     return  0;
    if( Color == "orange")  return  1;
    if( Color == "yellow")  return  2;
    if( Color == "green")   return  3;
    if( Color == "teal")    return  4;
    if( Color == "blue")    return  5;
    if( Color == "purple")  return  6;
    if( Color == "brown")   return  7;
    if( Color == "grey")    return  8;
    if( Color == "white")   return  9;
    if( Color == "black")   return  10;
    return -1;
}