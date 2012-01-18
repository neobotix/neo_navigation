/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Neobotix GmbH
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Neobotix nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/


#include <ros/ros.h>
#include <iostream>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>

/*\brief TestVelAcc is a wrapper class for converting the drive output of the SerialRelayBoard into ROS sensor_msgs;
*/
class TestVelAcc
{
	public:
	TestVelAcc();
	ros::NodeHandle n;
	ros::Publisher topicPub_vel;
	ros::Publisher topicPub_acc;	
	ros::Subscriber topicSub_pose;
	
	int init();
	void sendVel(const geometry_msgs::PoseWithCovarianceStamped&  c_pose);

	private:
	ros::Time now, last;
	geometry_msgs::PoseWithCovarianceStamped lastPose;
	geometry_msgs::Twist lastVel;

};


int TestVelAcc::init()
{
	now = ros::Time::now();
	last = now;
	topicSub_pose = n.subscribe("/amcl_pose",1,&TestVelAcc::sendVel, this);
	topicPub_vel = n.advertise<geometry_msgs::Twist>("/testVel",1);
	topicPub_acc = n.advertise<geometry_msgs::Twist>("/testAcc",1);
	ROS_INFO("started velocity test");
	return 0;
}


void TestVelAcc::sendVel(const geometry_msgs::PoseWithCovarianceStamped&  c_pose)
{
	now = ros::Time::now();
	double dt = (now-last).toSec();
	last = now;
	//ROS_INFO("wrap velocity cmd: %f %f",newState.velocities[0],newState.velocities[1]);
	geometry_msgs::Twist v;

	v.linear.x = (c_pose.pose.pose.position.x - lastPose.pose.pose.position.x)/dt;
	v.linear.y = (c_pose.pose.pose.position.y - lastPose.pose.pose.position.y)/dt;
	v.linear.z = sqrt(pow(v.linear.x,2) + pow(v.linear.y,2));
	//v.angular.z =
	topicPub_vel.publish(v);
	geometry_msgs::Twist a;
	a.linear.x = (v.linear.x - lastVel.linear.x) / dt;
	a.linear.y = (v.linear.y - lastVel.linear.y) / dt;
	a.linear.z = sqrt(pow(a.linear.x,2) + pow(a.linear.y,2));
	//a.angular.z
	topicPub_acc.publish(a);
	lastPose = c_pose;
	lastVel = v;
}

/**
*/
TestVelAcc::TestVelAcc () 
{


}



/**
*/
int main (int argc, char** argv)
{
	ros::init(argc, argv, "neo_SRBDrive_node");
	
	TestVelAcc  node;
	if(node.init() != 0) return 1;
	ros::spin();
	return 0;
}
