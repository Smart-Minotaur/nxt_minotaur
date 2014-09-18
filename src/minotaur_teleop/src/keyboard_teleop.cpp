#include <exception>
#include <ros/ros.h>
#include <termios.h>
#include <signal.h>
#include "minotaur_common/MinotaurControlNode.hpp"
#include "minotaur_common/Thread.hpp"

#define NODE_NAME "keyboard_teleop"

#define KEYCODE_R 0x43
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_Q 0x71

#define MAX_LIN_VEL 0.15f
#define MAX_ANG_VEl 1.0f

namespace minotaur
{
	class KeyboardThread : public Thread
	{
	private:
		MinotaurControlNode &controlNode;
		volatile bool keepRunning;
		struct termios oldtio, currtio;
		
	protected:
		void onStart() {
			keepRunning = true;
		}

		void onStop() {
			keepRunning = false;
		}

		void initKeyHandling() {
			//save for restore
			tcgetattr(0, &oldtio);

			// put the console in raw mode to catch key events
			tcgetattr(0, &currtio);
			currtio.c_lflag &= ~(ICANON | ECHO);
			tcsetattr(0, TCSANOW, &currtio);
		}

	public:
		KeyboardThread(MinotaurControlNode &p_controlNode)
			:controlNode(p_controlNode) {
			initKeyHandling();
		}

		~KeyboardThread() {
			tcsetattr(0, TCSANOW, &oldtio);
		}

		void run() {
			char keycode;
			ros::Rate loop_rate(60);

			ROS_INFO("Starting Eventloop...");

			while(keepRunning) {
				// read key events
				if(read(0, &keycode, 1) < 0) {
					perror("read():");
					exit(-1);
				}

				switch(keycode) {
				case KEYCODE_L:
					controlNode.setVelocity(0, MAX_ANG_VEl);
					break;
				case KEYCODE_R:
					controlNode.setVelocity(0, -MAX_ANG_VEl);
					break;
				case KEYCODE_U:
					controlNode.setVelocity(MAX_LIN_VEL, 0);
					break;
				case KEYCODE_D:
					controlNode.setVelocity(-MAX_LIN_VEL, 0);
					break;
				case KEYCODE_Q:
					controlNode.setVelocity(0,0);
					break;
				}

				loop_rate.sleep();
			}

		}
	};

	class KeyboardTeleop
	{
	private:
		ros::NodeHandle handle;
		MinotaurControlNode controlNode;
		KeyboardThread *thread;

	public:
		KeyboardTeleop()
			:handle(), controlNode() {
			controlNode.connectToROS(handle);
			thread = new KeyboardThread(controlNode);
		}

		~KeyboardTeleop() {
			delete thread;
		}

		void run() {
			thread->start();
			controlNode.spin();
		}

		void stop() {
			controlNode.stop();
			thread->stop();
		}
	};
}

static minotaur::KeyboardTeleop *teleop;
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
		minotaur::KeyboardTeleop teleop_;
		teleop = &teleop_;
		setSignalAction();
		teleop->run();
	} catch(std::exception &e) {
		ROS_ERROR("Exception: %s.", e.what());
	} catch(...) {
		ROS_ERROR("Caught unknown.");
	}
	ROS_INFO("Shutdown.");
	ros::shutdown();

	return 0;
}
