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
CDataEntity::CDataEntity(const int xmin, const int ymin, const int xmax, const int ymax){
	BoundingBoxInitialised = false;
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
	if ((xmax - xmin) > 0 && (ymax - ymin) > 0 && xmin > 0 && ymin > 0 && xmax > 0 && ymax > 0){
		BoundingBoxInitialised = true;
		this->Box.xmin = xmin;
		this->Box.ymin = ymin;
		this->Box.xmax = xmax;
		this->Box.ymax = ymax;
	} else {
		BoundingBoxInitialised = false;
		this->Box.xmin = 0;
		this->Box.ymin = 0;
		this->Box.xmax = 0;
		this->Box.ymax = 0;
	}
	return BoundingBoxInitialised;
}

bool CDataEntity::isBoundingBoxInitialised() const {
	return BoundingBoxInitialised;
}
