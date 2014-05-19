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
      // sensor2
      setSensorDistances(2, 0, 0);
      // sensor3
      setSensorDistances(3, 0, 0);
    }
  
    MapCreator::MapCreator(int width, int height)
    {
      map.setDimension(width, height);
      
      // set Default-values for sensors:
      setSensorDistances(1, 0, 0);
      // sensor2
      setSensorDistances(2, 0, 0);
      // sensor3
      setSensorDistances(3, 0, 0);
      
    }
    
    MapCreator::MapCreator(Map p_map)
    {
      map = p_map;
      
      // set Default-values for sensors:
      setSensorDistances(1, 0, 0);
      // sensor2
      setSensorDistances(2, 0, 0);
      // sensor3
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

    void MapCreator::step(int sensorValue1, int sensorValue2, int sensorValue3)
    {
      calculateObstaclePosition(sensorValue1, 1);
      calculateObstaclePosition(sensorValue2, 2);
      calculateObstaclePosition(sensorValue3, 3);
    }
    
    void MapCreator::step(const int p_sensor, const int p_distance)
    {
        calculateObstaclePosition(p_distance, p_sensor);
    }

    void MapCreator::calculateObstaclePosition(int measuredDistance, const int sensor)
    {
      float distance_X;
      float distance_Y;
      float angle;
      float dif_angle;
      float realDistance;
      float tmp_dist;
      int position_x, position_y;
      int ** gri = map.getField();
      int * height = NULL;

      if (measuredDistance >= 100) {
	ROS_WARN("MapCreator: Measured Distance too high: %dcm.",measuredDistance);
	return;
      }
	
      if (sensor == 1) {
	realDistance = measuredDistance + sen1.y;
	angle = pos.theta;
	
      } else if (sensor == 2) {
	tmp_dist = measuredDistance + sen2.x;
	realDistance = sqrt((tmp_dist * tmp_dist) + (sen2.y * sen2.y));
	dif_angle = asin( sen2.y / realDistance);
	angle = (pos.theta + 90.0) - dif_angle;
	if (angle > 360.0) {
	  angle = angle - 360.0;
	} else if (angle < 0.0) {
	  angle = angle + 360.0;
	}
	
      } else {
	tmp_dist = measuredDistance + sen3.x;
	realDistance = sqrt((tmp_dist * tmp_dist) + (sen3.y * sen3.y));
	dif_angle = asin ( sen3.y / realDistance );
	angle = (pos.theta - 90.0) - dif_angle;
	if (angle > 360.0) {
	  angle = angle - 360.0;
	} else if (angle < 0.0) {
	  angle = angle + 360.0;
	}
      }
      
      distance_X = sin(angle * DEGREE_TO_RAD) * realDistance;
      distance_Y = cos(angle * DEGREE_TO_RAD) * realDistance;

      position_x = (int) (pos.x + distance_X);
      position_y = (int) (pos.y + distance_Y);
      
      if ( position_x > 500 || position_x < 0 || position_y < 0 || position_y > 500) {
	ROS_WARN("MapCreator: Value out of Area: (%d/%d).",position_x, position_y );
	return;
      }
      
      height = gri[position_x];
      height[position_y]++;
      
      printf("Inc cell [%d] [%d] to %d -- dist: %2.6f\n", position_y, position_x, map.getField()[position_y][position_x], realDistance);
      
    }
    
    Map MapCreator::getMap()
    {
      return map;
    }

    // TODO: take map instead of grid / remove grid
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