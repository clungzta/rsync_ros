#!/usr/bin/env python
#Based upon actionlib client tutorial
#http://wiki.ros.org/actionlib_tutorials/Tutorials/Writing%20a%20Simple%20Action%20Client%20%28Python%29

import rospy
import roslib; roslib.load_manifest('rsync_ros')
import actionlib
import rsync_ros.msg
import sys

def rsync_client():

    # Creates the SimpleActionClient, passing the type of the action
    # (RsyncAction) to the constructor.
    client = actionlib.SimpleActionClient('rsync_ros', rsync_ros.msg.RsyncAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal = rsync_ros.msg.RsyncGoal(rsync_args=sys.argv[1:-2], source_path=sys.argv[-2], destination_path=sys.argv[-1])

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()  # A RsyncResult

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('rsync_client')
        result = rsync_client()
        print "Successful Transfer: {}".format(result.sync_success)

    except rospy.ROSInterruptException:
        print "program interrupted before completion"