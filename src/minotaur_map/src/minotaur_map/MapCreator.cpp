#include <signal.h>
#include <unistd.h>
#include <cmath>
#include <iostream>
#include <stdio.h>
#include <fstream>

#include "minotaur_map/MapCreator.hpp"
#include "minotaur_common/Math.hpp"

#define CONE_VALUES 7

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

const float coneAngles[] = {15, 10, 5, 0, -5, -10, -15};

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
        int tmpX = (int) meterToCm(x);
        int tmpY = (int) meterToCm(y);
        switch(sensor)
        {
            case MAP_CREATOR_FRONT_SENSOR: 
                sen1.x = tmpX;
                sen1.y = tmpY;
                break;
            case MAP_CREATOR_RIGHT_SENSOR:
                sen2.x = tmpX;
                sen2.y = tmpY;
                break;
            case MAP_CREATOR_LEFT_SENSOR:
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
        pos.x = meterToCm(p_position.point.x) + getXOffset();
        pos.y = meterToCm(p_position.point.y) + getYOffset();
        pos.theta = radianToDegree(p_position.theta);
    }
    
    void MapCreator::step(const int p_sensor, const int p_distance)
    {
        calculateObstaclePosition(p_sensor, p_distance);
    }
    
    void MapCreator::calculateObstaclePosition(const int sensor, int measuredDistance)
    {
      
      float angle;
      float tmp_angle;
      float realDistance;
      float tmp_dist;
      int positions_x[CONE_VALUES];
      int positions_y[CONE_VALUES];
      int *pos_x = positions_x;
      int *pos_y = positions_y;
      int i;

      if (measuredDistance >= MAX_DISTANCE) {
	return;
      }
      if (measuredDistance <= 0) {
	ROS_WARN("MapCreator: Invalid measured distance for sensor%d: %dcm. Value ignored", sensor, measuredDistance);
	return;
      }
	
	// sensor1 
      if (sensor == MAP_CREATOR_FRONT_SENSOR) {
	realDistance = measuredDistance + sen1.x;
	angle = pos.theta;
      
	// sensor 2
      } else if (sensor == MAP_CREATOR_RIGHT_SENSOR) {
	tmp_dist = measuredDistance + sen2.y;
	realDistance = sqrt((tmp_dist * tmp_dist) + (sen2.x * sen2.x));
	tmp_angle = asin( sen2.x / realDistance);
	angle = (pos.theta + 270.0) - radianToDegree(tmp_angle);
	
	// sensor 3
      } else if (sensor == MAP_CREATOR_LEFT_SENSOR) {
	tmp_dist = measuredDistance + sen3.y;
	realDistance = sqrt((tmp_dist * tmp_dist) + (sen3.x * sen3.x));
	tmp_angle = asin ( sen3.x / realDistance );
	angle = (pos.theta + 90.0) + radianToDegree(tmp_angle);
      
      } else {
	ROS_WARN("MapCreator: Got an invalid sensor");
	return;
      }
      
      angle = checkAngle(angle);
      generateCone(angle, realDistance, pos_x, pos_y);
      if (checkForInvalidValues(pos_x, pos_y)) {
        return;
      }
      
      for(i = 0; i < CONE_VALUES; i++){
	if(map.getField()[pos_x[i]][pos_y[i]] != INT_MAX){
	  incrementCells(pos_x[i], pos_y[i], i);
	  if (i == 3) {
 	    // Debug-output
	  }
	    
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
    
    bool MapCreator::checkForInvalidValues(int *p_pos_x, int *p_pos_y)
    {
      int i;
      for(i = 0; i < CONE_VALUES; i++){
	if (p_pos_x[i] > (map.getWidth() - 1) || p_pos_x[i] < 0 || p_pos_y[i] < 0 || p_pos_y[i] > (map.getHeight() - 1)) {
	  return true;
	}
      }
      return false;
	
    }
    
    void MapCreator::generateCone(float p_angle, float p_realDistance, int *p_pos_x, int *p_pos_y)
    {
      float distances_x[CONE_VALUES];
      float distances_y[CONE_VALUES];
      float *dist_x = distances_x;
      float *dist_y = distances_y;
      float tmp_angle;
      int i;
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
      p_dist_x[index] = cos(degreeToRadian(p_angle)) * p_realDistance;
      p_dist_y[index] = sin(degreeToRadian(p_angle)) * p_realDistance;
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
