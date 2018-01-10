/*
 * Field.hpp
 *
 *  Created on: Oct 5, 2014
 *      Author: rafael
 */

#ifndef FIELD_HPP
#define FIELD_HPP

class Field {
public:
	Field();
	virtual ~Field();

	float getBoundaryDepth() const;
	void setBoundaryDepth(float boundaryDepth);
	float getBoundaryWidth() const;
	void setBoundaryWidth(float boundaryWidth);
	float getCircleRadius() const;
	void setCircleRadius(float circleRadius);
	float getDepth() const;
	void setDepth(float depth);
	float getGoalDepth() const;
	void setGoalDepth(float goalDepth);
	float getGoalWidth() const;
	void setGoalWidth(float goalWidth);
	float getKeeperAreaDepth() const;
	void setKeeperAreaDepth(float keeperAreaDepth);
	float getKeeperAreaWidth() const;
	void setKeeperAreaWidth(float keeperAreaWidth);
	float getWidth() const;
	void setWidth(float width);
	float getAreaDepth() const;
	void setAreaDepth(float areaDepth);
	float getAreaWidth() const;
	void setAreaWidth(float areaWidth);

private:
	float depth, width, boundaryDepth, boundaryWidth, goalDepth, goalWidth, areaDepth, areaWidth, keeperAreaDepth, keeperAreaWidth, circleRadius;
};

#endif /* FIELD_HPP */

