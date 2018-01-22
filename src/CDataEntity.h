//
// Created by lucas on 21/01/18.
//

#ifndef PROJECT_CDATAENTITY_H
#define PROJECT_CDATAENTITY_H

#include "wm_data_collector/BoundingBox.h"

class CDataEntity {
public:
	CDataEntity(int xmin, int ymin, int xmax, int ymax);
	bool setBoundingBox(int xmin, int ymin, int xmax, int ymax);
	bool isBoundingBoxInitialised() const;

private:
	wm_data_collector::BoundingBox Box;
	bool BoundingBoxInitialised;

};


#endif //PROJECT_CDATAENTITY_H
