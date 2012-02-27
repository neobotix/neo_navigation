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
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_listener.h>


class MultiGoals
{
	public:
	MultiGoals() : timeAtGoal(1), stuckTime(10) {};
	ros::NodeHandle n;
	ros::Subscriber subs_path;
	ros::Subscriber subs_wayPoint;
	ros::Subscriber subs_robotOdom;
	ros::Subscriber subs_time;
	ros::Subscriber subs_clearWayPoints;
	ros::Publisher pub_navGoal;
	ros::Publisher pub_plan;
	ros::Publisher pub_cmd_vel;
	ros::Publisher pub_deltaWay;

	int init();
	
	void addWayPoint(const geometry_msgs::PoseStamped& pose);
	void setPath(const nav_msgs::Path& np);
	void setSleepTime(const std_msgs::Float32 dt);
	void getRobotOdom(const nav_msgs::Odometry& odom);
	void clearWayPoints(const std_msgs::Empty& clear);
	void setNewNavGoal();
	void checkForNextGoal();
	void tfDeltaPose();
	
	
	private:
	nav_msgs::Path path;
	ros::Time now, last;
	ros::Duration timeAtGoal;
	ros::Duration stuckTime;
	int currentGoal;
	int count;
	bool activePlan;
	bool reachedGoal;
	bool stuck;
	double dx, dy;
	double lPoseX, lPoseY, deltaWay;
	bool hasPose;
	tf::TransformListener listener;
	geometry_msgs::Twist twist;

};

int MultiGoals::init()
{
	count = 0;
	path.header.frame_id="/map";
	dy = 10000; dx = 10000;
	currentGoal = 0;
	activePlan = false;
	reachedGoal = false;
	stuck = false;
	hasPose = false;
	deltaWay = 0;
	subs_clearWayPoints = n.subscribe("/clear_way_points",1,&MultiGoals::clearWayPoints, this);
	subs_path = n.subscribe("/multi_goal_path",1,&MultiGoals::setPath, this);
	subs_wayPoint = n.subscribe("/add_way_point",1,&MultiGoals::addWayPoint, this);
	subs_robotOdom = n.subscribe("/odom",1,&MultiGoals::getRobotOdom, this);
	subs_time = n.subscribe("/goal_sleep_time",1,&MultiGoals::setSleepTime, this);
	pub_navGoal = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal",1);
	pub_plan = n.advertise<nav_msgs::Path>("/get_multi_path",1);
	pub_cmd_vel = n.advertise<geometry_msgs::Twist>("/cmd_vel",1);
	pub_deltaWay = n.advertise<std_msgs::Float32>("deltaWay",1);
	return 0;
}

void MultiGoals::addWayPoint(const geometry_msgs::PoseStamped& pose)
{
	path.poses.push_back(pose);
	if(!activePlan)
	{
		activePlan = true;
		setNewNavGoal();
	}
	pub_plan.publish(path);
};

void MultiGoals::clearWayPoints(const std_msgs::Empty& clear)
{
	activePlan = false;
	path.poses.clear();
	pub_plan.publish(path);	
};

void MultiGoals::setPath(const nav_msgs::Path& np)
{
	path = np;
	if(path.poses.size() == 0)
	{
		activePlan = false;
	}
	else
	{
		activePlan = true;
		currentGoal = path.poses.size(); //next goal will be the first in the path plan
		setNewNavGoal();
	}
	pub_plan.publish(path);
};

void MultiGoals::setSleepTime(const std_msgs::Float32 dt)
{
	ros::Duration deltaT(dt.data);
	timeAtGoal = deltaT;
};

void MultiGoals::getRobotOdom(const nav_msgs::Odometry& odom)
{
	twist = odom.twist.twist;
};

void MultiGoals::setNewNavGoal()
{
	currentGoal++;
	if(currentGoal >= path.poses.size() )
	{
		if(path.poses.size() == 0) return;
		currentGoal = 0;
	}

	ROS_DEBUG("setting new goal %i", currentGoal);
	pub_navGoal.publish(path.poses[currentGoal]);
};

void MultiGoals::checkForNextGoal()
{
	if(activePlan)
	{
		if(dy * dy + dx * dx < 0.4 && twist.linear.x < 0.005 && twist.linear.y < 0.005 && twist.angular.z < 0.005) //close to goal and not moving
		{
			if(reachedGoal)
			{
				now = ros::Time::now();
				if(now - last > timeAtGoal)
				{
					setNewNavGoal();
					usleep(10000);
					geometry_msgs::Twist twist;
					pub_cmd_vel.publish(twist); //zero robots velocity while replanning
				}
			}
			else 
			{
				last = ros::Time::now();
				reachedGoal = true;
			}
		} 
		else 
		{
			reachedGoal = false;
			if(twist.linear.x < 0.005 && twist.linear.y < 0.005 && twist.angular.z < 0.005)
			{	//stuck?
				if(!stuck)
				{
					stuck = true;
					last = ros::Time::now();
				}
				else
				{
					now = ros::Time::now();
					if(now - last > stuckTime ) //definitifly stuck, replan...
					{
						pub_navGoal.publish(path.poses[currentGoal]);
						usleep(10000);
						geometry_msgs::Twist twist;
						pub_cmd_vel.publish(twist); //zero robots velocity while replanning
						ROS_DEBUG("resetting goal %i", currentGoal);
						stuck = false; //wait another stuckTime till next reset
					}
				}
			}
			else stuck = false;
		}
	}
};

void MultiGoals::tfDeltaPose()
{
	tf::StampedTransform transform;
	try{
	listener.lookupTransform("/map", "/base_link",  
		               ros::Time(0), transform);
	}
	catch (tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
	}
	if(activePlan) //make sure path.poses[i] doesn't get called uninitialized...
	{
		dx = transform.getOrigin().x() - path.poses[currentGoal].pose.position.x;
		dy = transform.getOrigin().y() - path.poses[currentGoal].pose.position.y;
		if(!hasPose)
		{
			lPoseX = transform.getOrigin().x();
			lPoseY = transform.getOrigin().y();
			hasPose = true;
		}
		else
		{
			double dx_ = transform.getOrigin().x() - lPoseX;
			double dy_ = transform.getOrigin().y() - lPoseY;
			deltaWay += sqrt( dx_ * dx_ + dy_ * dy_  );
			count++;
			if(count > 100)
			{
				count = 0;
				std_msgs::Float32 dW;
				dW.data = deltaWay;
				pub_deltaWay.publish(dW);
			}
			lPoseX = transform.getOrigin().x();
			lPoseY = transform.getOrigin().y();
		}
	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "multi_goal_navigation");
	MultiGoals goals;
	goals.init();
	ros::Rate r(2);
	while(	ros::ok() )
	{
		goals.checkForNextGoal();
		goals.tfDeltaPose();
		ros::spinOnce();
	}
	return 0;
}
