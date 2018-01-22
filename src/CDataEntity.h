//
// Created by lucas on 21/01/18.
//

#ifndef PROJECT_CDATAENTITY_H
#define PROJECT_CDATAENTITY_H

#include "ros/ros.h"
#include <sstream>
#include "wm_color_detector/BoundingBox.h"

class CDataEntity {
public:
	CDataEntity(const int xmin, const int ymin, const int xmax, const int ymax, const std::string name);
	bool setBoundingBox(int xmin, int ymin, int xmax, int ymax);
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
