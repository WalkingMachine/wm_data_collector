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
    if( Color == "Red")     return  0;
    if( Color == "Orange")  return  1;
    if( Color == "Yellow")  return  2;
    if( Color == "Green")   return  3;
    if( Color == "Cyan")    return  4;
    if( Color == "Blue")    return  5;
    if( Color == "Magenta") return  6;
    if( Color == "Brown")   return  7;
    if( Color == "Grey")    return  8;
    if( Color == "White")   return  9;
    if( Color == "Black")   return  10;
    return -1;
}