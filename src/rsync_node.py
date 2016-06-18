#!/usr/bin/env python
import roslib
import rospy
roslib.load_manifest('rsync_actionlib')
from subprocess import Popen, PIPE
import actionlib
from rsync_ros.msg import RsyncAction, RsyncResult

class RsyncServer:
  def __init__(self):
    self.server = actionlib.SimpleActionServer('rsync', RsyncAction, self.execute, False)
    self.server.start()

  def execute(self, goal):
  	command = ['rsync'] + goal.rsync_args + [goal.source_path, goal.destination_path]

	rospy.loginfo("Executing rsync command '%s'", ' '.join(command))

	p = Popen(command,shell=False,stdout=PIPE,stderr=PIPE)
	stdout, stderr = p.communicate()
	rospy.loginfo(stdout)
	
	if stderr:
		rospy.logerr(stderr)
	
	result = RsyncResult()
	result.sync_success = p.returncode > -1

	rospy.loginfo("Rsync command result '%s'", result.sync_success)

	self.server.set_succeeded(result)

if __name__ == "__main__":
	try:
		rospy.init_node('rsync_actionlib')
		rospy.loginfo("Ready to sync files.")
		RsyncServer()
		rospy.spin()
	except rospy.ROSInterruptException:
		pass
