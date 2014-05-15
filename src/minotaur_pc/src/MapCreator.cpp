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

int grid[500][500];


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
      switch(sensor)
      {
	case 1: 
	  sen1.x = x;
	  sen1.y = y;
	  break;
	case 2:
	  sen2.x = x;
	  sen2.y = y;
	  break;
	case 3:
	  sen3.x = x;
	  sen3.y = y;	
	  break;
	default:
	  //ROS_INFO("invalid sensor-number");
	  break;
      }
    }

    void MapCreator::setPosition(RobotPosition p_position)
    {
      pos.x = p_position.point.x;
      pos.y = p_position.point.y;
      pos.theta = p_position.theta * RAD_TO_DEGREE;
    }

    void MapCreator::step(int sensorValue1, int sensorValue2, int sensorValue3)
    {
      calculateObstaclePosition(sensorValue1, 1);
      calculateObstaclePosition(sensorValue2, 2);
      calculateObstaclePosition(sensorValue3, 3);
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
	//ROS_INFO("----No valid Value----");
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
	//ROS_INFO("----Value out of Area----");
	return;
      }
      
      height = gri[position_x];
      height[position_y]++;
      
      grid[position_y][position_x]++;
      printf("Inc cell [%d] [%d] to %d -- dist: %2.6f\n", position_y, position_x, grid[position_y][position_x], realDistance);
      
    }
    
    Map MapCreator::getMap()
    {
      return map;
    }

    // TODO: take map instead of grid / remove grid
    void MapCreator::createTextFile()
    {
      int x, y;
      const char *path="grid.txt";
      ofstream file;
      file.open (path);
      for ( y = 499; y >= 0; y-- ) {
	for ( x = 0; x <= 499; x++ ) {
	  if ( grid[y][x] == 0 ) {
	    file << "-";
	  } else if ( grid[y][x] > 9) {
	    file << "9";
	  } else {
	    file << grid[y][x];
	  }
	}
	file << "\n";
      }
      file.close();
    }
}