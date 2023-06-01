#include<ros/ros.h>
#include<std_msgs/Float64.h>
#include <geometry_msgs/Point.h>
#include "dynamixel_msgs/JointState.h"
#include "dynamixel_msgs/MotorStateList.h"
#include<stdio.h>
#include <stdlib.h>
#include <math.h>
#include <fcntl.h>
#include <sys/time.h>
#include <time.h>
#include <fstream>



double getcurrtime() {  //function to calculate current time
	struct timeval tv;
	gettimeofday(&tv,NULL);
	//printf("Time: %d %d\n", tv.tv_sec, tv.tv_usec);
	return (double)tv.tv_sec + (double)tv.tv_usec / 1.0e6;
}


using namespace std;

class Ball
{
public:
	static Ball *_singleton;
	ros::Subscriber sub;
	double px, py, vx, vy, currtime;
	double prevx, prevy, prevtime;

	Ball() {
		_singleton = this;
		px = py = 0;
	}

	void subscribe(ros::NodeHandle& n) {
		sub = n.subscribe("/point", 1, pointCallback); //subscribing to a topic called point
	}

	static void pointCallback(const geometry_msgs::Point::ConstPtr& msg)
	{
		float x = float(msg->x - 160) ;
		float y = float(msg->y - 120) ;

		
		
		//std::cout << "Point: " << msg->x << ", " << msg->y << std::endl;
		Ball &This = *_singleton;
		This.prevx = This.px;
		This.prevy = This.py;
		This.prevtime = This.currtime;
		This.px = x*0.75/1000; //getting actual posiition in 'm' from pixel data
		This.py = 0.23 + (y*0.791667)/1000;
		This.currtime = getcurrtime();
		//printf("Cam time: %f\n", This.currtime);

		float t = This.currtime - This.prevtime;
		if (t == 0) return;
		double alpha = 0.4;
		This.vx = This.vx * alpha + ((This.px - This.prevx) / t) * (1 - alpha); //calculating velocity from position information
		This.vy = This.vy * alpha + ((This.py - This.prevy) / t) * (1 - alpha);
	}
} ball;
Ball *Ball::_singleton = NULL;

class RobotInterface
{
public:
	struct Joint {
		double min, max, mean;
		double velocity;
		double position;
		double currTime;
		double prevPosition;
		double prevTime;
		double acceleration;
		double prevVelocity;
		const char *name;

		Joint() {
			acceleration = 0;
			velocity = 0;
			position = 0;
			currTime = 0;
			prevPosition = 0;
			prevTime = 0;
			prevVelocity = 0;
		}
	};

	static RobotInterface *_singleton;
	Joint joint_pitch, joint_roll;
	ros::Subscriber sub1, sub2;
	std::ofstream fout;

RobotInterface() {
		_singleton = this;
		joint_pitch.name = "joint_pitch";
		joint_pitch.min = -0.40;
		joint_pitch.mean = 1.57;
		joint_pitch.max = 0.40;
		joint_roll.name = "joint_roll";
		joint_roll.min = -0.70;
		joint_roll.mean = 3.14;
		joint_roll.max = 0.45;
		fout.open("state.log");
	}
	

	void subscribeState(ros::NodeHandle& n) {
		sub1 = n.subscribe("/wrist_pitch_position_controller/state", 1, stateCallback1);
		sub2 = n.subscribe("/wrist_roll_position_controller/state", 1, stateCallback2);
	}

	static void stateCallback(const dynamixel_msgs::JointState::ConstPtr& msg, Joint& joint) {
		long s = (long)msg->header.stamp.sec;
		long ns = (long)msg->header.stamp.nsec;
		double t = s + (double)ns / 1000000000.0;

		_singleton->fout << msg->header.stamp.sec << 
			'\t' << msg->header.stamp.nsec <<
			'\t' << joint.name << 
			'\t' << msg->current_pos << 
			'\t' << msg->velocity << 
			std::endl;
		

		joint.prevPosition = joint.position;
		joint.prevTime = joint.currTime;
		
		double filter = 0.5;


		joint.position = msg->current_pos - joint.mean;
		joint.currTime = t;

		joint.prevVelocity = joint.velocity;
		joint.velocity = joint.velocity * filter + ((joint.position - joint.prevPosition) / (joint.currTime - joint.prevTime)) * (1-filter);
		joint.acceleration = joint.acceleration*filter + ((joint.velocity - joint.prevVelocity) / (joint.currTime - joint.prevTime)) * (1-filter);
		}

	static void stateCallback1(const dynamixel_msgs::JointState::ConstPtr& msg)
	{
		stateCallback(msg, _singleton->joint_pitch);
	}

	static void stateCallback2(const dynamixel_msgs::JointState::ConstPtr& msg)
	{
		stateCallback(msg, _singleton->joint_roll);
	}
} robot;

RobotInterface *RobotInterface::_singleton = NULL;

int main(int argc,char** argv)
{
	std::ofstream log1("state1.log");
	ros::init(argc, argv, "robot_balance");
	ros::NodeHandle n;
	ros::Publisher pub1 = n.advertise<std_msgs::Float64>("/wrist_pitch_position_controller/command", 1);
	ros::Publisher pub2 = n.advertise<std_msgs::Float64>("/wrist_roll_position_controller/command", 1);
	ros::Rate r(100); // Hertz
	robot.subscribeState(n);
	ball.subscribe(n);

	int t = 0;
	double a1 = 0, a2 = 0, dt = 0, ct = 0, pt = 0, input1 = 0, input2 = 0;
	double elapsedtime = 0, currenttime=0, prevtime=0, inittime = 0, prevvel1 = 0;
	double vel1 = 0, vel2 = 0;
	double pos2;
	bool firstLog1 = true;
	inittime = getcurrtime();
	while(ros::ok())
	{
		




		//printf("(%f, %f)\n", robot.joint_pitch.velocity, robot.joint_roll.velocity);
		//printf("(%f, %f)\n", robot.joint_pitch.position, robot.joint_roll.position);
		//printf("PointVel (%f, %f)\n", ball.vx, ball.vy);
		//printf("Pointpos (%f, %f)\n", ball.px, ball.py);
		//printf("(%f, %f)\n", robot.joint_pitch.acceleration, robot.joint_roll.acceleration);

		a1 = 30*ball.vy-20*robot.joint_pitch.velocity-83.75*robot.joint_pitch.position+ 50*(ball.py - 0.23);
		//a1 = 29.43*ball.vy-15*robot.joint_pitch.velocity-83.75*robot.joint_pitch.position+26.97*ball.py - 6.20;
		//a2 = 29.43*ball.vx-15*robot.joint_roll.velocity-83.75*robot.joint_roll.position+26.97*ball.px;
  		a2 = 29.43*ball.vx-15*robot.joint_roll.velocity-83.75*robot.joint_roll.position+35*ball.px;
		
		currenttime = getcurrtime() - inittime;
		dt = 0.01;
		
		if (robot.joint_pitch.position <= robot.joint_pitch.min || robot.joint_pitch.position >= robot.joint_pitch.max)
		{ 
		vel1 = 0;
		}
		else 
		{

		vel1 = robot.joint_pitch.velocity + a1*dt;
		input1 = vel1 + 10*(vel1 - robot.joint_pitch.velocity);

		}

		if (robot.joint_roll.position <= robot.joint_roll.min || robot.joint_roll.position >= robot.joint_roll.max)
		{
		vel2 = 0;
		}

		else
		{

		vel2 = robot.joint_roll.velocity + a2*dt;
		input2 =  vel2 + 14*(vel2 - robot.joint_roll.velocity);
	
		
		}
		
		
		std_msgs::Float64 msg1;
		msg1.data = input1;
		//msg1.data = 0.02;		
		pub1.publish(msg1);
		

		std_msgs::Float64 msg2;
		msg2.data = input2;
		pub2.publish(msg2);
		

		//printf("FinalVel (%f , %f)\n", vel1, vel2);
		//printf("time (%f)\n", dt);
		//printf("acceleration (%f , %f)\n", a1, a2);
		
		// Comment out the following to only display values and not publish to robot

		

		printf("(%f, %f)\n", robot.joint_pitch.velocity, vel1);
		//printf("FinalVel (%f, %f)\n", vel1, vel2);
		//printf("(%f, %f)\n", robot.joint_roll.velocity, robot.joint_roll.position);
		//printf("PointVel (%f, %f)\n", ball.vx, ball.vy);
		printf("Pointpos (%f, %f)\n", ball.vy, ball.py);
		//printf("time (%f)\n", dt);
		//printf("acceleration (%f , %f)\n", a1, a2);

		if (firstLog1) {
			firstLog1 = false;
			log1 << "time,PitchVelocity,RollVelocity,PitchPosition,RollPosition,BallVX,BallVY,BallPX,BallPY,FinalVel1,FinalVel2,DT,A1,A2" << std::endl;
		}
		log1 << currenttime << "," << robot.joint_pitch.velocity << "," << robot.joint_roll.velocity << ","
			<< robot.joint_pitch.position << "," << robot.joint_roll.position << ","
			<< ball.vx << "," << ball.vy << "," 
			<< ball.px << "," << ball.py << ","
			<< vel1 << "," << vel2 << ","
			<< dt << "," 
			<< a1 << "," << a2 << 
			std::endl;


		ros::spinOnce();		
		r.sleep();
	}
	return 0;
}

