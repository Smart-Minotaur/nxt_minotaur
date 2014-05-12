#ifndef MAPCREATOR_HPP_
#define MAPCREATOR_HPP_

#include "minotaur_pc/Map.hpp"

#define RAD_TO_DEGREE 57.295779513082
#define DEGREE_TO_RAD 0.017453292519943

namespace minotaur
{
  class MapCreator
  {
  private:
    Map map;
    int grid[500][500];
  public:
    MapCreator();
    ~MapCreator();
    void setSensorDistances(int sensor, int x, int y);
    void step(int sensorValue1, int sensorValue2, int sensorValue3);
    void calculateObstaclePosition(int measuredDistance, const int sensor);
    void setPosition(double x, double y, double currentDirection);
    void createTextFile();
  };
}

#endif
