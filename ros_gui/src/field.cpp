/*
 * Field.cpp
 *
 *  Created on: Oct 5, 2014
 *      Author: rafael
 */

#include "../include/ros_gui/field.hpp"

Field::Field() {
	// TODO Auto-generated constructor stub
    setBoundaryDepth(105);
    setBoundaryWidth(200);
    setDepth(100);
    setWidth(180);
    setGoalDepth(30);
    setGoalWidth(10);
    setCircleRadius(20);
    setKeeperAreaDepth(20);
    setKeeperAreaWidth(10);
    setAreaDepth(40);
    setAreaWidth(20);
}

Field::~Field() {
	// TODO Auto-generated destructor stub
}

float Field::getBoundaryDepth() const {
	return boundaryDepth;
}

void Field::setBoundaryDepth(float boundaryDepth) {
	this->boundaryDepth = boundaryDepth;
}

float Field::getBoundaryWidth() const {
	return boundaryWidth;
}

void Field::setBoundaryWidth(float boundaryWidth) {
	this->boundaryWidth = boundaryWidth;
}

float Field::getCircleRadius() const {
	return circleRadius;
}

void Field::setCircleRadius(float circleRadius) {
	this->circleRadius = circleRadius;
}

float Field::getDepth() const {
	return depth;
}

void Field::setDepth(float depth) {
	this->depth = depth;
}

float Field::getGoalDepth() const {
	return goalDepth;
}

void Field::setGoalDepth(float goalDepth) {
	this->goalDepth = goalDepth;
}

float Field::getGoalWidth() const {
	return goalWidth;
}

void Field::setGoalWidth(float goalWidth) {
	this->goalWidth = goalWidth;
}

float Field::getKeeperAreaDepth() const {
	return keeperAreaDepth;
}

void Field::setKeeperAreaDepth(float keeperAreaDepth) {
	this->keeperAreaDepth = keeperAreaDepth;
}

float Field::getKeeperAreaWidth() const {
	return keeperAreaWidth;
}

void Field::setKeeperAreaWidth(float keeperAreaWidth) {
	this->keeperAreaWidth = keeperAreaWidth;
}

float Field::getWidth() const {
	return width;
}

void Field::setWidth(float width) {
	this->width = width;
}

float Field::getAreaDepth() const {
	return areaDepth;
}

void Field::setAreaDepth(float areaDepth) {
	this->areaDepth = areaDepth;
}

float Field::getAreaWidth() const {
	return areaWidth;
}

void Field::setAreaWidth(float areaWidth) {
	this->areaWidth = areaWidth;
}
