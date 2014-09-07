#include "mouse_monitor_pc/MouseMonitorCalibrationData.hpp"

namespace minotaur
{

MouseMonitorCalibrationData::MouseMonitorCalibrationData()
{
	distance = 0;

	s1XDisplacement = 0.0;
	s1YDisplacement = 0.0;
	s2XDisplacement = 0.0;
	s2YDisplacement = 0.0;

	s1AngleOffset = 0.0;
	s2AngleOffset = 0.0;
	s1RealDistance = 0.0;
	s2RealDistance = 0.0;
}

MouseMonitorCalibrationData::~MouseMonitorCalibrationData()
{
}

void MouseMonitorCalibrationData::setS1AngleOffset(double angleOffset) {
	this->s1AngleOffset = angleOffset;
}
void MouseMonitorCalibrationData::setS2AngleOffset(double angleOffset) {
	this->s2AngleOffset = angleOffset;
}
void MouseMonitorCalibrationData::setDistance(int distance) {
	this->distance = distance;
}
void MouseMonitorCalibrationData::setS1RealDistance(double realDistance) {
	this->s1RealDistance = realDistance;
}
void MouseMonitorCalibrationData::setS2RealDistance(double realDistance) {
	this->s2RealDistance = realDistance;
}
void MouseMonitorCalibrationData::setS1XDisplacement(double s1XDisplacement) {
	this->s1XDisplacement = s1XDisplacement;
}
void MouseMonitorCalibrationData::setS1YDisplacement(double s1YDisplacement) {
	this->s1YDisplacement = s1YDisplacement;
}
void MouseMonitorCalibrationData::setS2XDisplacement(double s2XDisplacement) {
	this->s2XDisplacement = s2XDisplacement;
}
void MouseMonitorCalibrationData::setS2YDisplacement(double s2YDisplacement) {
	this->s2YDisplacement = s2YDisplacement;
}
double MouseMonitorCalibrationData::getS1AngleOffset() const {
	return s1AngleOffset;
}
double MouseMonitorCalibrationData::getS2AngleOffset() const {
	return s2AngleOffset;
}
int MouseMonitorCalibrationData::getDistance() const {
	return distance;
}
double MouseMonitorCalibrationData::getS1RealDistance() const {
	return s1RealDistance;
}
double MouseMonitorCalibrationData::getS2RealDistance() const {
	return s2RealDistance;
}
double MouseMonitorCalibrationData::getS1XDisplacement() const {
	return s1XDisplacement;
}
double MouseMonitorCalibrationData::getS1YDisplacement() const {
	return s1YDisplacement;
}
double MouseMonitorCalibrationData::getS2XDisplacement() const {
	return s2XDisplacement;
}
double MouseMonitorCalibrationData::getS2YDisplacement() const {
	return s2YDisplacement;
}

}

