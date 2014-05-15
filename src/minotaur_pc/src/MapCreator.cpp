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
#define SENSOR_FRONT 1
#define SENSOR_RIGHT 2
#define SENSOR_LEFT 3

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

int grid_old[500][500];


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
	  //ROS_INFO("invalid sensor-number: Use 1, 2 or 3");
	  break;
      }
    }

    void MapCreator::setPosition(RobotPosition p_position)
    {
      pos.x = p_position.point.x * 100 + (map.getWidth() /2); // offset um 0-0 als Mittelpunkt zu nehmen
      pos.y = p_position.point.y * 100 + (map.getHeight() /2);
      pos.theta = p_position.theta * RAD_TO_DEGREE;
    }

    void MapCreator::step(int sensorValue1, int sensorValue2, int sensorValue3)
    {
      calculateObstaclePosition(sensorValue1, SENSOR_FRONT);
      calculateObstaclePosition(sensorValue2, SENSOR_RIGHT);
      calculateObstaclePosition(sensorValue3, SENSOR_LEFT);
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
	
      if (sensor == SENSOR_FRONT) {
	realDistance = measuredDistance + sen1.y;
	angle = pos.theta;
	
      } else if (sensor == SENSOR_RIGHT) {
	tmp_dist = measuredDistance + sen2.x;
	realDistance = sqrt((tmp_dist * tmp_dist) + (sen2.y * sen2.y));
	// TODO: fix calculation
	dif_angle = asin( sen2.y / realDistance);
	angle = (pos.theta + 90.0) - dif_angle;
	if (angle > 360.0) {
	  angle = angle - 360.0;
	} else if (angle < 0.0) {
	  angle = angle + 360.0;
	}
	
      } else if (sensor == SENSOR_LEFT) {
	tmp_dist = measuredDistance + sen3.x;
	realDistance = sqrt((tmp_dist * tmp_dist) + (sen3.y * sen3.y));
	dif_angle = asin ( sen3.y / realDistance );
	angle = (pos.theta - 90.0) - dif_angle;
	if (angle > 360.0) {
	  angle = angle - 360.0;
	} else if (angle < 0.0) {
	  angle = angle + 360.0;
	}
      } else {
	return;
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
      
      grid_old[position_y][position_x]++;
      printf("Inc cell [%d] [%d] to %d -- dist: %2.6f\n", position_y, position_x, grid_old[position_y][position_x], realDistance);
      
    }
    
    Map MapCreator::getMap()
    {
      return map;
    }

    void MapCreator::createTextFile()
    {
      int x, y;
      int ** grid = map.getField();
      int * field = NULL;
      const char *path="grid.txt";
      ofstream file;
      file.open (path);
      for ( y = map.getHeight()-1; y >= 0; y-- ) {
	field = grid[y];
	for ( x = 0; x <= map.getWidth()-1; x++ ) {
	  if ( field[x] == 0 ) {
	    file << "-";
	  } else if ( field[x] > 9) {
	    file << "9";
	  } else {
	    file << field[x];
	  }
	}
	file << "\n";
      }
      file.close();
    }
}