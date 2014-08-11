#ifndef MINOTAUR_MAP_MAPCREATOR_HPP_
#define MINOTAUR_MAP_MAPCREATOR_HPP_

#include "minotaur_map/Map.hpp"
#include "minotaur_map/RobotPosition.hpp"

#define MAP_CREATOR_LEFT_SENSOR 3
#define MAP_CREATOR_RIGHT_SENSOR 2
#define MAP_CREATOR_FRONT_SENSOR 1

#define CONE_VAL_15 1
#define CONE_VAL_10 2
#define CONE_VAL_5 4
#define CONE_VAL_0 8

#define MAX_DISTANCE 30

namespace minotaur
{

    /**
     * \brief Calculates the histogramm values.
     *
     * This class manages and processes a Map (histogramm field) object
     * and calculates the values of the cells of the map.
     */
    class MapCreator
    {
    private:
        Map map;
    public:
        MapCreator();
        MapCreator(int width, int height);
        MapCreator(Map p_map);
        ~MapCreator();
        
        /**
         * Sets the distances of the sensors to the mid of the robot.
         * These values are considered during step() to calculate which
         * cells have to be incremented.
         * @param sensor index of sensor (left,right,front)
         * @param x distance to robot in x-direction (vertically)
         * @param y distance to robot in y-direction (horizontally)
         */
        void setSensorDistances(int sensor, float x, float y);
        
        /**
         * Uses the given sensor data to increment the values in the
         * histogramm.
         * @param p_sensor index of sensor (left,right,front)
         * @param p_distance measured distance in cm
         */
        void step(const int p_sensor, const int p_distance);
        void calculateObstaclePosition(const int sensor, int measuredDistance);
        void incrementCells(int position_y, int position_x, int quality);
        void generateCone(float p_angle, float p_realDistance, int *p_pos_x, int *p_pos_y);
        float checkAngle(float p_angle);
        bool checkForInvalidValues(int *p_pos_x, int *p_pos_y);
        void calculateDistances(float p_angle, float p_realDistance, float *p_dist_x, float *p_dist_y, int index);
        
        /**
         * Sets the position of the robot. This information is needed 
         * during step() to calculate which cells have to be incremented.
         * @param p_position current position of the robot.
         */
        void setPosition(RobotPosition p_position);
        
        /**
         * Grants access to the Map object containing the histogramm.
         * @return pointer to the Map object
         */
        Map* getMap();
        void createTextFile(const char *path);

        int getXOffset();
        int getYOffset();
    };
}

#endif
