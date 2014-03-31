#include <signal.h>
#include <termios.h>
#include <poll.h>
#include "ros/ros.h"
#include "nxt_minotaur/nxtPower.h"

#define KEYCODE_R 0x43
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_Q 0x71

ros::Publisher powerPub;
int kfd = 0;
struct termios oldtio, currtio;
struct sigaction sa;

void setSignalAction();
void sighandler(int sig);
void sendMsg(nxt_minotaur::nxtPower msg);
void initKeyHandling();
void eventLoop();

int main(int argc, char** argv)
{
    ros::init(argc, argv, "nxtTeleop");
    
    setSignalAction();
    
    ros::NodeHandle n;
    
    powerPub = n.advertise<nxt_minotaur::nxtPower>("cmd_pow", 1000);
    
    initKeyHandling();
    eventLoop();
    
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
    tcsetattr(0, TCSANOW, &oldtio);
    ros::shutdown();
    exit(0);
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
    nxt_minotaur::nxtPower msg;
    msg.leftMotor = 0;
    msg.rightMotor = 0;
    
    while(true)
    {
        if(read(kfd, &keycode, 1) < 0)
        {
            perror("read():");
            exit(-1);
        }

        switch(keycode)
        {
            case KEYCODE_L:
                ROS_DEBUG("LEFT");
                msg.leftMotor = -100;
                msg.rightMotor = 100;
                sendMsg(msg);
                break;
            case KEYCODE_R:
                ROS_DEBUG("RIGHT");
                msg.leftMotor = 100;
                msg.rightMotor = -100;
                sendMsg(msg);
                break;
            case KEYCODE_U:
                ROS_DEBUG("UP");
                msg.leftMotor = 100;
                msg.rightMotor = 100;
                sendMsg(msg);
                break;
            case KEYCODE_D:
                ROS_DEBUG("DOWN");
                msg.leftMotor = -100;
                msg.rightMotor = -100;
                sendMsg(msg);
                break;
            case KEYCODE_Q:
                ROS_DEBUG("STOP");
                msg.leftMotor = 0;
                msg.rightMotor = 0;
                sendMsg(msg);
                break;
        }
            
        loop_rate.sleep();
    }
}

void sendMsg(nxt_minotaur::nxtPower msg)
{
    powerPub.publish(msg);
    ros::spinOnce();
}

