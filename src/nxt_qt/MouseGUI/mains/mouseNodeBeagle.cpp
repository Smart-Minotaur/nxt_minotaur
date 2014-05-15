#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nxt_beagle/Config.hpp"
//#include "nxt_beagle/RobotController.hpp"
//#include "nxt_beagle/MVelocity.h"
#include "nxt_qt/DebugMouseData.h"
#include "/home/benni/teamprojekt/trunk/src/IPLNTrackingDevice.h"
//#include "/home/benni/teamprojekt/trunk/src/PLN2033.h"
#include "/home/benni/teamprojekt/trunk/src/PLN2033.h" ../../../trunk/build/*.a


#include <sstream>

/*nxt_beagle::MVelocity motorVelocityToMsg(const minotaur::MotorVelocity& p_velocity)
{
    nxt_beagle::MVelocity result;
    result.leftVelocity = p_velocity.leftMPS;
    result.rightVelocity = p_velocity.rightMPS;
    return result;
}*/

pln_minotaur::IPLNTrackingDevice *sensor1;
pln_minotaur::IPLNTrackingDevice *sensor2;


int main(int argc, char **argv)
{

  ros::init(argc, argv, "MOUSENODEBEAGLE");
  ros::NodeHandle n;

  ros::Publisher mouseData_pub = n.advertise<nxt_qt::DebugMouseData>("mouseDebug", 1000);
  ros::Rate loop_rate(100);
  
  
  pln_minotaur::PLN2033_Settings settings1;
  pln_minotaur::PLN2033_Settings settings2;
  
  sensor1 = new pln_minotaur::PLN2033("/dev/spidev1.0");
  sensor2 = new pln_minotaur::PLN2033("/dev/spidev1.1");

  
  
  double count = 0.0;
  while (ros::ok())
  {
    nxt_qt::DebugMouseData mouse;
    
    
    
    /*pln_minotaur::readStatusAndSpeed( mouse.mouse1_x_speed, mouse.mouse1_y_speed );
    ROS_INFO("X: %f, Y: %f, Speed X: %f, Speed Y: %f", 
	      mouse.mouse1_x_disp, mouse.mouse1_y_disp, mouse.mouse1_x_speed, mouse.mouse1_y_speed);
    */
    
//read sensor registers
  settings1 = sensor1->readPLNSettings();
  settings2 = sensor2->readPLNSettings();
  
  ROS_INFO("SENSOR1: reading registers");
  mouse.mouse1_status_register = settings1.status_register;
  mouse.mouse1_delta_x_disp_register = settings1.delta_x_disp_register;
  mouse.mouse1_delta_y_disp_register = settings1.delta_y_disp_register;
  mouse.mouse1_command_high_register = settings1.command_high_register;
  mouse.mouse1_command_low_register = settings1.command_low_register;
  mouse.mouse1_memory_pointer_register = settings1.memory_pointer_register;
  mouse.mouse1_memory_data_register = settings1.memory_data_register;
  mouse.mouse1_mode_control_register = settings1.mode_control_register;
  mouse.mouse1_power_control_register = settings1.power_control_register;
  mouse.mouse1_mode_status_register = settings1.mode_status_register;
  mouse.mouse1_system_control_register = settings1.system_control_register;
  mouse.mouse1_miscellaneous_register = settings1.miscellaneous_register;
  mouse.mouse1_interrupt_output_register = settings1.interrupt_output_register;
  
  ROS_INFO("SENSOR2: reading registers");
  mouse.mouse2_status_register = settings2.status_register;
  mouse.mouse2_delta_x_disp_register = settings2.delta_x_disp_register;
  mouse.mouse2_delta_y_disp_register = settings2.delta_y_disp_register;
  mouse.mouse2_command_high_register = settings2.command_high_register;
  mouse.mouse2_command_low_register = settings2.command_low_register;
  mouse.mouse2_memory_pointer_register = settings2.memory_pointer_register;
  mouse.mouse2_memory_data_register = settings2.memory_data_register;
  mouse.mouse2_mode_control_register = settings2.mode_control_register;
  mouse.mouse2_power_control_register = settings2.power_control_register;
  mouse.mouse2_mode_status_register = settings2.mode_status_register;
  mouse.mouse2_system_control_register = settings2.system_control_register;
  mouse.mouse2_miscellaneous_register = settings2.miscellaneous_register;
  mouse.mouse2_interrupt_output_register = settings2.interrupt_output_register;
    
    
//read sonsor movement values
    if (sensor1->readStatusAndDisplacementAndSpeed(mouse.mouse1_x_speed, mouse.mouse1_y_speed,
      mouse.mouse1_x_disp, mouse.mouse1_y_disp))
    {
      //ROS_INFO("SENSOR1: Speed X: %f, Speed Y: %f, [X: %f, Y: %f]", mouse.mouse1_x_speed, mouse.mouse1_y_speed, 
	//       mouse.mouse1_x_disp, mouse.mouse1_y_disp);
    }
    else
    {
      mouse.mouse1_x_speed = 100.0;
      mouse.mouse1_y_speed = 200.0;
      mouse.mouse1_x_disp = -100.12 + count;
      mouse.mouse1_y_disp = 100.34;
    }
    
    
    if (sensor2->readStatusAndDisplacementAndSpeed(mouse.mouse1_x_speed, mouse.mouse1_y_speed,
      mouse.mouse1_x_disp, mouse.mouse1_y_disp))
    {
      //ROS_INFO("SENSOR2: Speed X: %f, Speed Y: %f, [X: %f, Y: %f]", mouse.mouse1_x_speed, mouse.mouse1_y_speed, 
	//       mouse.mouse1_x_disp, mouse.mouse1_y_disp);
    }
    else
    {
      mouse.mouse2_x_speed = 300.0;
      mouse.mouse2_y_speed = 400.0;
      mouse.mouse2_x_disp = 100.12 - count;
      mouse.mouse2_y_disp = -100.34;
    }
    
    
    ROS_INFO("SENSOR1: Speed X: %f, Speed Y: %f, [X: %f, Y: %f]", mouse.mouse1_x_speed, mouse.mouse1_y_speed, 
	       mouse.mouse1_x_disp, mouse.mouse1_y_disp);
    ROS_INFO("SENSOR2: Speed X: %f, Speed Y: %f, [X: %f, Y: %f]", mouse.mouse2_x_speed, mouse.mouse2_y_speed, 
	       mouse.mouse2_x_disp, mouse.mouse2_y_disp);
    

    mouseData_pub.publish(mouse);

    ros::spinOnce();

    loop_rate.sleep();
    if(count == 200.0) 
    {
      count = 0.0;
      
    }
    ++count;
  }


  return 0;
}
