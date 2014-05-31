/*
 * Author: Fabian Meyer 
 */

#include <signal.h>
#include <termios.h>
#include <poll.h>
#include "ros/ros.h"
#include "robot_control/Utils.hpp"
#include "nxt_control/Motor.hpp"
#include "nxt_control/Brick.hpp"
#include "nxt_control/NxtOpcodes.hpp"

#define LEFT_PORT PORT_A
#define RIGHT_PORT PORT_B

#define KEYCODE_R 0x43
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_Q 0x71

nxtcon::Brick brick;
nxtcon::Motor leftMotor;
nxtcon::Motor rightMotor;

struct termios oldtio, currtio;
struct sigaction sa;
bool run = true;

void setSignalAction();
void sighandler(int sig);
bool initBrick();
void initKeyHandling();
void eventLoop();

int main(int argc, char** argv)
{
    ros::init(argc, argv, "nxtTeleop");
    ros::Time::init();
    
    setSignalAction();
    
   if(!initBrick())
    return -1;
    
    initKeyHandling();
    eventLoop();
    ros::shutdown();
    
    return 0;
}

void setSignalAction()
{
    memset(&sa, 0, sizeof(struct sigaction));
    sa.sa_handler = sighandler;
    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGQUIT, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);
}

void sighandler(int sig)
{
    run = false;
}

bool initBrick()
{
     try
    {
        ROS_INFO("Initialize USBConnection to Brick...");
        brick.find();
        leftMotor.setBrick(&brick);
        leftMotor.setPort(LEFT_PORT);
        
        rightMotor.setBrick(&brick);
        rightMotor.setPort(RIGHT_PORT);
        
        return true;
    }
    catch (std::exception const &e)
    {
        ROS_ERROR("Exception on initializing Brick: %s.", e.what());
        return false;
    }
}

void initKeyHandling()
{
    //save for restore
    tcgetattr(0, &oldtio);
    
    // put the console in raw mode
    tcgetattr(0, &currtio);
    currtio.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(0, TCSANOW, &currtio);

    puts("Reading from keyboard");
    puts("---------------------------");
    puts("Use arrow keys to move Bender.");
}


void eventLoop()
{
    char keycode;
    ros::Rate loop_rate(60);
    
    while(run)
    {
        if(read(0, &keycode, 1) < 0)
        {
            perror("read():");
            break;
        }

        switch(keycode)
        {
            case KEYCODE_L:
                ROS_DEBUG("LEFT");
                leftMotor.setPower(-100);
                rightMotor.setPower(100);
                break;
            case KEYCODE_R:
                ROS_DEBUG("RIGHT");
                leftMotor.setPower(100);
                rightMotor.setPower(-100);
                break;
            case KEYCODE_U:
                ROS_DEBUG("UP");
                leftMotor.setPower(100);
                rightMotor.setPower(100);
                break;
            case KEYCODE_D:
                ROS_DEBUG("DOWN");
                leftMotor.setPower(-100);
                rightMotor.setPower(-100);
                break;
            case KEYCODE_Q:
                ROS_DEBUG("STOP");
                leftMotor.brake();
                rightMotor.brake();
                break;
        }
            
        loop_rate.sleep();
    }
    
    tcsetattr(0, TCSANOW, &oldtio);
    ROS_INFO("Eventloop terminated.");
}

