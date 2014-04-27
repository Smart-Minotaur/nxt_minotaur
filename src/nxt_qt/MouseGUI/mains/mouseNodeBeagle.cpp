#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nxt_beagle/Config.hpp"
#include "nxt_beagle/RobotController.hpp"
#include "nxt_beagle/MVelocity.h"
#include "nxt_qt/DebugMouse.h"

#include <sstream>

nxt_beagle::MVelocity motorVelocityToMsg(const minotaur::MotorVelocity& p_velocity)
{
    nxt_beagle::MVelocity result;
    result.leftVelocity = p_velocity.leftMPS;
    result.rightVelocity = p_velocity.rightMPS;
    return result;
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "talker");
  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<nxt_qt::DebugMouse>("chatter", 1000);
  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok())
  {
    std_msgs::String msg;
	//nxt_beagle::RVelocity test(const minotaur::MotorVelocity& p_velocity);
	//nxt_beagle::MVelocity msgM;

	nxt_qt::DebugMouse mouse;
	mouse.MouseXValue = -100 + count;
	mouse.MouseYValue = 100;

	//msgM.leftVelocity = 50;
	//msgM.rightVelocity = 49;
	

    //ROS_INFO("%s %d", msg.data.c_str(), 123);
	//ROS_INFO("X: %f, Y: %f", msgM.leftVelocity, msgM.rightVelocity);
	ROS_INFO("X: %d, Y: %d", mouse.MouseXValue, mouse.MouseYValue);
	ROS_INFO("Count: %d", count);

    chatter_pub.publish(mouse);

    ros::spinOnce();

    loop_rate.sleep();
    if(count == 200) 
    {
      count = 0;
      
    }
    ++count;
  }


  return 0;
}
