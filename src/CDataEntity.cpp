//
// Created by lucas on 21/01/18.
//

#include "CDataEntity.h"

/**
 * Initialise a DataEntity object with a bounding box
 * @param xmin	abscissa of the first point of the bounding box
 * @param ymin	ordinate of the first point of the bounding box
 * @param xmax	abscissa of the last point of the bounding box
 * @param ymax	ordinate of the last point of the bounding box
 */
CDataEntity::CDataEntity(const int xmin, const int ymin, const int xmax, const int ymax, const std::string name) {
	this->BoundingBoxInitialised = false;
	this->name = name;
	this->color = "unknown";
	setBoundingBox(xmin, ymin, xmax, ymax);
}

/**
 * Initialise the bounding box of the entity
 * @param xmin	abscissa of the first point of the bounding box
 * @param ymin	ordinate of the first point of the bounding box
 * @param xmax	abscissa of the last point of the bounding box
 * @param ymax	ordinate of the last point of the bounding box
 * @return 	if the bounding box was correctly initialised
 */
bool CDataEntity::setBoundingBox(const int xmin, const int ymin, const int xmax, const int ymax) {
	// Test if the bounding box is a correct bounding box.
	if ((xmax - xmin) > 0 && (ymax - ymin) > 0 && xmin >= 0 && ymin >= 0 && xmax > 0 && ymax > 0) {
		this->BoundingBoxInitialised = true;
		this->box.xmin = xmin;
		this->box.ymin = ymin;
		this->box.xmax = xmax;
		this->box.ymax = ymax;
	} else {
		this->BoundingBoxInitialised = false;
		this->box.xmin = 0;
		this->box.ymin = 0;
		this->box.xmax = 0;
		this->box.ymax = 0;
	}
	return this->BoundingBoxInitialised;
}

/**
 * Print all the data about the entity
 */
void CDataEntity::printEntity() {
	ROS_INFO("%s (%s) : %ix%i to %ix%i", name.c_str(), color.c_str(), box.xmin, box.ymin, box.xmax, box.ymax);
}

/** --- --- --- --- GETTERS --- --- --- --- **/
bool CDataEntity::isBoundingBoxInitialised() const {
	return BoundingBoxInitialised;
}
const wm_color_detector::BoundingBox &CDataEntity::getBox() const {
	return box;
}

/** --- --- --- --- SETTERS --- --- --- --- **/
void CDataEntity::setColor(const std::string &color) {
	CDataEntity::color = color;
}
