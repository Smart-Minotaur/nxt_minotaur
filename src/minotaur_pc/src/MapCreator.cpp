//#include <ros/ros.h>
#include <signal.h>
#include <unistd.h>
#include <cmath>
#include <iostream>
#include <stdio.h>
#include <fstream>

#include "minotaur_pc/MapCreator.hpp"

#define RAD_TO_DEGREE 57.295779513082
#define DEGREE_TO_RAD 0.017453292519943
#define METER_TO_CENTIMETER(m) (m * 100)
#define CONE_VALUES 7
#define CONE_VAL_15 1
#define CONE_VAL_10 2
#define CONE_VAL_5 3
#define CONE_VAL_0 4

#define MAX_DISTANCE 30

using namespace std;

struct sensorDistanceFromZero {
    int x;
    int y;
} sen1, sen2, sen3;

struct position {
    float x;
    float y;
    float theta;
} pos;

namespace minotaur {
    
    // Default Constructor
    MapCreator::MapCreator()
    {
        map.setDimension(500, 500);
        
        // set Default-values for sensors:
        setSensorDistances(1, 0, 0);
        setSensorDistances(2, 0, 0);
        setSensorDistances(3, 0, 0);
    }
    
    MapCreator::MapCreator(int width, int height)
    {
        map.setDimension(width, height);
        
        // set Default-values for sensors:
        setSensorDistances(1, 0, 0);
        setSensorDistances(2, 0, 0);
        setSensorDistances(3, 0, 0);
        
    }
    
    MapCreator::MapCreator(Map p_map)
    {
        map = p_map;
        
        // set Default-values for sensors:
        setSensorDistances(1, 0, 0);
        setSensorDistances(2, 0, 0);
        setSensorDistances(3, 0, 0);
        
    }
    
    MapCreator::~MapCreator()
    {
        
    }
    
    
    /**
     * Sets the Distance from Zero
     * sensor = Number of sensor
     *  -> 1 = frontsensor           
     *  -> 2 = sensor on the right
     *  -> 3 = sensor on the left
     * x = Distance in x direction
     * y = Distance in y direction
     */
    void MapCreator::setSensorDistances(int sensor, float x, float y)
    {
        int tmpX = (int) METER_TO_CENTIMETER(x);
        int tmpY = (int) METER_TO_CENTIMETER(y);
        switch(sensor)
        {
            case 1: 
                sen1.x = tmpX;
                sen1.y = tmpY;
                break;
            case 2:
                sen2.x = tmpX;
                sen2.y = tmpY;
                break;
            case 3:
                sen3.x = tmpX;
                sen3.y = tmpY;	
                break;
            default:
                ROS_WARN("MapCreator: invalid sensor number: %d.", sensor);
                break;
        }
    }
    
    void MapCreator::setPosition(RobotPosition p_position)
    {
        pos.x = METER_TO_CENTIMETER(p_position.point.x) + getXOffset();
        pos.y = METER_TO_CENTIMETER(p_position.point.y) + getYOffset();
        pos.theta = p_position.theta * RAD_TO_DEGREE;
    }
    
    void MapCreator::step(const int p_sensor, const int p_distance)
    {
        calculateObstaclePosition(p_sensor, p_distance);
    }
    
    void MapCreator::calculateObstaclePosition(const int sensor, int measuredDistance)
    {
      
      float angle = 0.0;
      float dif_angle;
      float realDistance;
      float tmp_dist;
      int positions_x[CONE_VALUES];
      int positions_y[CONE_VALUES];
      int *pos_x = positions_x;
      int *pos_y = positions_y;
      int position_x, position_y;
      int i;

      if (measuredDistance >= MAX_DISTANCE) {
	ROS_INFO("MapCreator: The mesaured distance for sensor%d is too damn high: %dcm. Value ignored", sensor, measuredDistance);
	return;
      }
      if (measuredDistance <= 0) {
	ROS_WARN("MapCreator: Invalid measured distance for sensor%d: %dcm. Value ignored", sensor, measuredDistance);
	return;
      }
	
      if (sensor == 1) {
	realDistance = measuredDistance + sen1.x;
	angle = pos.theta;
	
      } else if (sensor == 2) {
	tmp_dist = measuredDistance + sen2.y;
	realDistance = sqrt((tmp_dist * tmp_dist) + (sen2.x * sen2.x));
	dif_angle = asin( sen2.x / realDistance);
	angle = (pos.theta + 270.0) - (dif_angle * RAD_TO_DEGREE);
	
      } else if (sensor == 3) {
	tmp_dist = measuredDistance + sen3.y;
	realDistance = sqrt((tmp_dist * tmp_dist) + (sen3.x * sen3.x));
	dif_angle = asin ( sen3.x / realDistance );
	angle = (pos.theta + 90.0) + (dif_angle * RAD_TO_DEGREE);
      } else {
	ROS_WARN("MapCreator: Got an invalid sensor");
	return;
      }
      
      angle = checkAngle(angle);
      getCone(angle, realDistance, pos_x, pos_y);

      if ( position_x > (map.getWidth() -1 ) || position_x < 0 || position_y < 0 || position_y > (map.getHeight() -1 )) {
	ROS_WARN("----Value out of Area----");
	return;
      }
      
      for(i = 0; i < CONE_VALUES; i++){
	if(map.getField()[pos_x[i]][pos_y[i]] != INT_MAX){
	  incrementCells(pos_x[i], pos_y[i], i);
	  if (i == 3)
 	    ROS_INFO("MapCreator: Sensor%d: Inc cell [%d] [%d] to %d\n", sensor, pos_x[i], pos_y[i], map.getField()[pos_x[i]][pos_y[i]]);
	}
      }
    }
    
    float MapCreator::checkAngle(float p_angle)
    // keep angle between 0 and 360 degrees
    {
      if (p_angle > 360.0) {
	  p_angle = p_angle - 360.0;
      } else if (p_angle < 0.0) {
	  p_angle = p_angle + 360.0;
      }
      return p_angle;
    }
    
    void MapCreator::getCone(float p_angle, float p_realDistance, int *p_pos_x, int *p_pos_y)
    {
      float distances_x[CONE_VALUES];
      float distances_y[CONE_VALUES];
      float *dist_x = distances_x;
      float *dist_y = distances_y;
      float tmp_angle;
      const float coneAngles[] = {15, 10, 5, 0, -5, -10, -15};
      int i, angleCase;
      for(i = 0; i < CONE_VALUES; i++) {
	tmp_angle = p_angle + coneAngles[i];
	tmp_angle = checkAngle(tmp_angle);
	calculateDistances(tmp_angle, p_realDistance, dist_x, dist_y, i);
	p_pos_x[i] = (int) (pos.x + distances_x[i]);
	p_pos_y[i] = (int) (pos.y + distances_y[i]);
      }
    }
    
    void MapCreator::calculateDistances(float p_angle, float p_realDistance, float *p_dist_x, float *p_dist_y, int index)
    {
      p_dist_x[index] = cos((p_angle) * DEGREE_TO_RAD) * p_realDistance;
      p_dist_y[index] = sin((p_angle) * DEGREE_TO_RAD) * p_realDistance;
    }
    
    void MapCreator::incrementCells(int position_x, int position_y, int quality)
    {
      // The values in the middle of the cone got a higher value
      switch(quality)
      {
	case 0:
	case 6:
	  map.getField()[position_x][position_y] = map.getField()[position_x][position_y] + CONE_VAL_15;
	  break;
	case 1:
	case 5:
	  map.getField()[position_x][position_y] = map.getField()[position_x][position_y] + CONE_VAL_10;
	  break;
	case 2:
	case 4:
	  map.getField()[position_x][position_y] = map.getField()[position_x][position_y] + CONE_VAL_5;
	  break;
	case 3:
	  map.getField()[position_x][position_y] = map.getField()[position_x][position_y] + CONE_VAL_0;
	  break;
      }
    }
    
    Map* MapCreator::getMap()
    {
        return &map;
    }
    
    void MapCreator::createTextFile(const char *path)
    {
      int x, y;
      ofstream file;
      file.open (path);
      for ( y = map.getHeight() - 1; y >= 0; y-- ) {
	for ( x = 0; x < map.getWidth(); x++ ) {
	  if ( map.getField()[x][y] == 0 ) {
	    file << "-";
	  } else if ( map.getField()[x][y] > 9) {
	    file << "9";
	  } else {
	    file << map.getField()[x][y];
	  }
	}
	file << endl;
      }
      file.close();
    }
    
    int MapCreator::getXOffset()
    {
        return map.getWidth() / 2;
    }
    
    int MapCreator::getYOffset()
    {
        return map.getHeight() / 2;
    }
}