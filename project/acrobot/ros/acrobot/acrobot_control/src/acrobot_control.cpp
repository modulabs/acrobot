#include "ros/ros.h"
#include <iostream>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#ifdef _WIN32
#include <Windows.h>
#else
#include <unistd.h>
#endif

// ������ Ŭ����
#include "Acrobot.h"

Vector2f vecQ(0, 0);
Vector2f vecQdot(0, 0);

#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include <sensor_msgs/ChannelFloat32.h>
#include <sensor_msgs/JointState.h>

// callback �Լ��� �ޱ� ���� ���� ������ ����
void listen_joint_state_callback(const sensor_msgs::JointState & state)
{
	// ROS_INFO("POSITION [0] value : %f", state.position[0]);
	vecQ(0) = state.position[0];
	vecQ(1) = state.position[1];
	vecQdot(0) = state.velocity[0];
	vecQdot(1) = state.velocity[1];
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "acrobot_control");
	ros::NodeHandle nh;
	ros::Rate rate(100);

	// target
	Vector2f vecDesQ(0, 0);
	Vector2f vecDesQdot(0, 0);
	Vector2f vecDesQddot(0, 0);

	// Robot
	Link link1(0.1710, 0.377790, 0.27948, 0.0027273, 0.01);
	Link link2(0.289, 0.388200, 0.32843, 0.003348, 0.01);


	Acrobot acrobot(link1, link2);
	acrobot.setTargetValue(vecDesQ, vecDesQdot, vecDesQddot);
	// LQR
	acrobot.setParameter4LQR(-650.4009, -289.0746, -287.1833, -140.0615);
	// Swing up
	acrobot.setParameter4SwingUp(2, 100, 50);

	// message 
	std_msgs::Float64 input;
	ros::Subscriber sub = nh.subscribe("/acrobot/joint_states", 1000, listen_joint_state_callback);
	ros::Publisher pub = nh.advertise<std_msgs::Float64>("/acrobot/joint2_torque_controller/command", 1000);
	
	while (ros::ok())
	{
		ros::spinOnce(); // call back �Լ� ó��
		rate.sleep();
		
		// ROS_INFO("vecQ : %f %f , vecQdot : %f %f ",vecQ(0),vecQ(1),vecQdot(0),vecQdot(1));
		input.data = acrobot.calcControlInput(vecQ, vecQdot);
		// ROS_INFO("input.data: %f ",input.data);
		pub.publish(input);


	}

	return 0;
}
// #endif
