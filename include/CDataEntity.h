//
// Created by lucas on 21/01/18.
//

#ifndef PROJECT_CDATAENTITY_H
#define PROJECT_CDATAENTITY_H

#include "ros/ros.h"
#include <sstream>
#include "wm_color_detector/BoundingBox.h"
//#include "wm_data_collector/BoundingBox.h"


class CDataEntity {
public:
	CDataEntity();
	bool setBoundingBox(int xmin, int ymin, int xmax, int ymax);
	bool setName(std::string);
	void printEntity();
	void setColor(const std::string &color);
	bool isBoundingBoxInitialised() const;
	const wm_color_detector::BoundingBox &getBox() const;

private:
	std::string name;
	std::string color;
	wm_color_detector::BoundingBox box;
	bool BoundingBoxInitialised;

};


#endif //PROJECT_CDATAENTITY_H
