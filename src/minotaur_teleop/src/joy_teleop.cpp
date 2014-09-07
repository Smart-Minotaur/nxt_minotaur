#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <signal.h>
#include "minotaur_common/MinotaurControlNode.hpp"

#define NODE_NAME "joy_teleop"
#define JOY_TOPIC "/joy"
#define MAX_ANG_VEL 1.5f
#define MAX_LIN_VEL 0.20f

#define VERTICAL_AXIS 1
#define HORIZONTAL_AXIS 0
#define B_BUTTON 1

namespace minotaur
{
	class JoyTeleop
	{
	private:
		ros::NodeHandle handle;
		ros::Subscriber joySubscriber;
		MinotaurControlNode controlNode;
		
		void joyCallback(const sensor_msgs::Joy::ConstPtr &p_joy)
		{
			if(p_joy->buttons[B_BUTTON]) {
				controlNode.setVelocity(0,0);
			}
			else {
				float linFactor = p_joy->axes[VERTICAL_AXIS];
				float angFactor = p_joy->axes[HORIZONTAL_AXIS];
				
				controlNode.setVelocity(linFactor * MAX_LIN_VEL, angFactor * MAX_ANG_VEL);
			}
		}
		
	public:
		JoyTeleop()
		:controlNode()
		{
			controlNode.connectToROS(handle);
			ROS_INFO("Subscribing to topic \"%s\".", JOY_TOPIC);
			joySubscriber = handle.subscribe<sensor_msgs::Joy>(JOY_TOPIC, 40, &JoyTeleop::joyCallback, this);
		}
		
		~JoyTeleop()
		{
			
		}
	
		void run()
		{
			ROS_INFO("System up and ready.");
			controlNode.spin();
		}
		
		void stop()
		{
			controlNode.stop();
		}
	};
}

static minotaur::JoyTeleop *teleop;
static struct sigaction sa;

static void sighandler(int sig)
{
	ROS_INFO("Stop teleop.");
    teleop->stop();
}

static void setSignalAction()
{
    memset(&sa, 0, sizeof(struct sigaction));
    sa.sa_handler = sighandler;
    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGQUIT, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, NODE_NAME);
	
	try {
		minotaur::JoyTeleop teleop_;
		teleop = &teleop_;
		setSignalAction();
		teleop->run();
	} catch (std::exception &e) {
		ROS_ERROR("Exception: %s.", e.what());
	} catch(...) {
		ROS_ERROR("Caught unknown.");
	}
	ROS_INFO("Shutdown.");
	ros::shutdown();
	
	return 0;
}