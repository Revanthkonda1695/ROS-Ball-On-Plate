#include<ros/ros.h>
#include<std_msgs/Float64.h>
#include <geometry_msgs/Point.h>
#include "dynamixel_msgs/JointState.h"
#include "dynamixel_msgs/MotorStateList.h"
#include<stdio.h>
#include <stdlib.h>
#include <math.h>
#include <fcntl.h>


using namespace std;

/*
class Dynamixel{
private:
	ros::NodeHandle n;
	ros::Publisher pub1, pub2;

public:
	Dynamixel();
	int moveMotor(double position, int index);
};

Dynamixel::Dynamixel() 
{
	pub_n = n.advertise<std_msgs::Float64>("/wrist_roll_position_controller/command",0);
	pub_t = n.advertise<std_msgs::Float64>("/wrist_pitch_position_controller/command",1);
		for (int i = 0; i < NUM_MOTORS; i++)
			pubs[i] = n.advertise<std_msgs::Float64>(motorConfigs[i].name,1);
}

int Dynamixel::moveMotor(double position, int index)
{
	std_msgs::Float64 aux;
	aux.data = position;
	pubs[index].publish(aux);
	return 1;
}*/

int main(int argc,char** argv)
{
	ros::init(argc, argv, "robot_balance");
	ros::NodeHandle n;
	ros::Publisher pub1 = n.advertise<std_msgs::Float64>("/pan_controller1/command", 1);
	ros::Rate r(100);

	//system("/home/khan/cyton_gamma_300_ROS-master/ROS_ws/src/standup.sh");

	int t = 0;
	while(ros::ok())
	{
		ros::spinOnce();

		double vel = 0.1 * sin(t++ * M_PI * 0.5);
		if (t >= 30) vel = 0;

		printf("Publishing velocity %f\n", vel);

		std_msgs::Float64 msg;
		msg.data = vel;
		pub1.publish(msg);

		r.sleep();
	}
	return 0;
}

