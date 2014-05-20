#ifndef MAPCREATOR_HPP_
#define MAPCREATOR_HPP_

#include "minotaur_pc/Map.hpp"
#include "minotaur_pc/RobotPosition.hpp"

#define MAP_CREATOR_LEFT_SENSOR 3
#define MAP_CREATOR_RIGHT_SENSOR 2
#define MAP_CREATOR_FRONT_SENSOR 1


namespace minotaur
{
  class MapCreator
  {
  private:
    Map map;
  public:
    MapCreator();
    MapCreator(int width, int height);
    MapCreator(Map p_map);
    ~MapCreator();
    void setSensorDistances(int sensor, float x, float y);
    void step(const int p_sensor, const int p_distance);
    void calculateObstaclePosition(const int sensor, int measuredDistance);
    void incrementCells(int position_y, int position_x, int quality);
    void getCone(float p_angle, float p_realDistance, int *p_pos_x, int *p_pos_y);
    float checkAngle(float p_angle);
    void calculateDistances(float p_angle, float p_realDistance, float *p_dist_x, float *p_dist_y, int index);
    void setPosition(RobotPosition p_position);
    Map getMap();
    void createTextFile(const char *path);
    
    int getXOffset();
    int getYOffset();
  };
}

#endif
