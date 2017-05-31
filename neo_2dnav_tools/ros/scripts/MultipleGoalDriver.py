#!/usr/bin/env python

import rospy
import actionlib
import time
from smach import State,StateMachine
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped

#waypoints = [
#    ['S1', (0.0, -1.0), (0.0, 0.0, 0.0, 1.0)],
#    ['S2', (2.0, -1.0), (0.0, 0.0, 0.0, 1.0)],
#    ['S3', (2.0, 1.0), (0.0, 0.0, 0.707, 0.707)],
#    ['S4', (4.12, 3.12), (0.0, 0.0, 0.383, 0.924)],
#    ['S5', (4.12, 3.12), (0.0, 0.0, 0.924, -0.383)],
#    ['S6', (0.0, 3.12), (0.0, 0.0, 1.0, 0.0)]
#]

waypoints = [
    ['S1', (0.0, 0.0), (0.0, 0.0, 0.0, 1.0)],
    ['S2', (4.0, 0.0), (0.0, 0.0, 0.0, 1.0)],
]

class Waypoint(State):
    def __init__(self, position, orientation):
        State.__init__(self, outcomes=['success'])

        #global pub
        # Get an action client
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

        # Define the goal
        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = '/map'
        self.goal.target_pose.pose.position.x = position[0]
        self.goal.target_pose.pose.position.y = position[1]
        self.goal.target_pose.pose.position.z = 0.0
        self.goal.target_pose.pose.orientation.x = orientation[0]
        self.goal.target_pose.pose.orientation.y = orientation[1]
        self.goal.target_pose.pose.orientation.z = orientation[2]
        self.goal.target_pose.pose.orientation.w = orientation[3]

        #publlish waypoint
        self.target_pose = PoseStamped()
        self.target_pose.header.frame_id = '/map'
        self.target_pose.pose.position.x = position[0]
        self.target_pose.pose.position.y = position[1]
        self.target_pose.pose.position.z = 0.0
	self.target_pose.pose.orientation.x = orientation[0]
	self.target_pose.pose.orientation.y = orientation[1]
	self.target_pose.pose.orientation.z = orientation[2]
	self.target_pose.pose.orientation.w = orientation[3]
        #pub.publish(self.point)

    def execute(self, userdata):
	global pub
	global seq
	self.target_pose.header.seq = seq
	self.target_pose.header.stamp = rospy.Time.now()
	pub.publish(self.target_pose)
	time.sleep(2)
	seq = seq + 1
        self.client.send_goal(self.goal)
        self.client.wait_for_result()
        return 'success'


if __name__ == '__main__':
    rospy.init_node('patrol')

    global pub
    pub = rospy.Publisher('/waypoints_visu', PoseStamped, queue_size=10)

    global seq
    seq = 0

    # publish waypoints for rviz visualization
    #point = PointStamped()
    #for a, w in enumerate(waypoints):
        #point.header.stamp = rospy.get_rostime()
    #    print("In der Schleife")
    #    point.header.frame_id = 'map'
    #    point.point.x = w[1][0]
    #    point.point.y = w[1][1]
    #    point.point.z = 0.0
    #    pub.publish(point)

    patrol = StateMachine('success')
    with patrol:
	for i,w in enumerate(waypoints):
	    StateMachine.add(w[0],
		             Waypoint(w[1], w[2]),
		             transitions={'success':waypoints[(i + 1) % \
		             len(waypoints)][0]})

    patrol.execute()