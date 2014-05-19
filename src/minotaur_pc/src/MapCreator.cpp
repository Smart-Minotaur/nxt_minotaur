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
    void MapCreator::setSensorDistances(int sensor, int x, int y)
    {
      int tmpX = METER_TO_CENTIMETER(x);
      int tmpY = METER_TO_CENTIMETER(y);
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
      float distance_x;
      float distance_y;
      float angle = 0.0;
      float dif_angle;
      float realDistance;
      float tmp_dist;
      int position_x, position_y;

      if (measuredDistance >= 100) {
	//ROS_WARN("MapCreator: The mesaured distance for sensor%d is too damn high: %dcm. Value ignored", sensor, measuredDistance);
	return;
      }
      if (measuredDistance <= 0) {
	ROS_WARN("MapCreator: Invalid measured distance for sensor%d: %dcm. Value ignored", sensor, measuredDistance);
	return;
      }
	
      if (sensor == 1) {
	realDistance = measuredDistance + sen1.y;
	angle = pos.theta;
	
      } else if (sensor == 2) {
	tmp_dist = measuredDistance + sen2.x;
	realDistance = sqrt((tmp_dist * tmp_dist) + (sen2.y * sen2.y));
	dif_angle = asin( sen2.y / realDistance);
	angle = (pos.theta + 90.0) + (dif_angle * RAD_TO_DEGREE);
	if (angle > 360.0) {
	  angle = angle - 360.0;
	} else if (angle < 0.0) {
	  angle = angle + 360.0;
	}
	
      } else if (sensor == 3) {
	tmp_dist = measuredDistance + sen3.x;
	realDistance = sqrt((tmp_dist * tmp_dist) + (sen3.y * sen3.y));
	dif_angle = asin ( sen3.y / realDistance );
	ROS_INFO("dif angle sensor%d: %2f", sensor, dif_angle);
	angle = (pos.theta - 90.0) - (dif_angle * RAD_TO_DEGREE);
	if (angle > 360.0) {
	  angle = angle - 360.0;
	} else if (angle < 0.0) {
	  angle = angle + 360.0;
	}
      } else {
	ROS_WARN("MapCreator: Got an invalid sensor");
	return;
      }
      ROS_INFO("sensor%d: %2f", sensor, angle);
      
      distance_x = sin(angle * DEGREE_TO_RAD) * realDistance;
      distance_y = cos(angle * DEGREE_TO_RAD) * realDistance;

      position_x = (int) (pos.x + distance_x);
      position_y = (int) (pos.y + distance_y);
      
      if ( position_x > map.getWidth() || position_x < 0 || position_y < 0 || position_y > map.getHeight()) {
	//ROS_WARN("MapCreator: Value out of Area: (%d/%d).",position_x, position_y);
	return;
      }
      
      if(map.getField()[position_y][position_x] != INT_MAX){
	map.getField()[position_y][position_x]++;
	//ROS_INFO("MapCreator: Inc cell [%d] [%d] to %d\n", position_y, position_x, map.getField()[position_y][position_x]);
      }
      
    }
    
    Map MapCreator::getMap()
    {
      return map;
    }

    void MapCreator::createTextFile(const char *path)
    {
      int x, y;
      ofstream file;
      file.open (path);
      for ( y = map.getHeight() - 1; y >= 0; y-- ) {
	for ( x = 0; x < map.getWidth(); x++ ) {
	  if ( map.getField()[y][x] == 0 ) {
	    file << "-";
	  } else if ( map.getField()[y][x] > 9) {
	    file << "9";
	  } else {
	    file << map.getField()[y][x];
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