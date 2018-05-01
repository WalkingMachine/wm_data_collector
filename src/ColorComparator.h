//
// Created by Philippe on 4/30/18.
//

#ifndef PROJECT_COLORCOMPARATOR_H
#define PROJECT_COLORCOMPARATOR_H


#include <string>

#define NBCOLORS  11

namespace ColorComparison{

    const float ColorProximityMatrix[]{
            /*              0,      1,      2,      3,      4,      5,      6,      7,      8,      9,      10   */
            /*              Red,    Orange, Yellow, Green,  Teal,   Blue,   Purple, Brown,  Grey,   White,  Black  */
            /* 0 Red    */  1,      0.8,    0.5,    0,      0,      0.5,    0.8,    0.6,    0.2,    0.2,    0.7,
            /* 1 Orange */  0.8,    1,      0.8,    0.5,    0,      0,      0.5,    0.8,    0.2,    0.3,    0.6,
            /* 2 Yellow */  0.5,    0.8,    1,      0.8,    0.5,    0,      0,      0.6,    0.5,    0.7,    0.5,
            /* 3 Green  */  0,      0.5,    0.8,    1,      0.8,    0.5,    0,      0.6,    0.5,    0.8,    0.7,
            /* 4 Teal   */  0,      0,      0.5,    0.8,    1,      0.8,    0.5,    0.6,    0.5,    0.8,    0.7,
            /* 5 Blue   */  0.2,    0,      0,      0.5,    0.8,    1,      0.8,    0.6,    0.5,    0.8,    0.8,
            /* 6 Purple */  0.8,    0.5,    0,      0,      0.6,    0.8,    1,      0.6,    0.5,    0.8,    0.8,
            /* 7 Brown  */  0.8,    0.8,    0.6,    0.6,    0,      0.6,    0.6,    1,      0.5,    0,      0.9,
            /* 8 Grey   */  0.6,    0.2,    0.5,    0.5,    0.5,    0.5,    0.5,    0.5,    1,      0.8,    0.8,
            /* 9 White  */  0.2,    0.3,    0.7,    0.8,    0.8,    0.8,    0.8,    0,      0.8,    1,      0,
            /*10 Black  */  0.7,    0.6,    0.5,    0.7,    0.7,    0.8,    0.8,    0.9,    0.8,    0,      1
    };

    float CompareColors(std::string Color1, std::string Color2);
    int GetColorID(std::string Color);

}


#endif //PROJECT_COLORCOMPARATOR_H
