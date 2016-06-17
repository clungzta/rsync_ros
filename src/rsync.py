#!/usr/bin/env python
import rospy
from subprocess import Popen, PIPE
from rsync_ros.srv import RsyncRequest
import sys
import threading

class RsyncThread(threading.Thread):
    def __init__(self):
        self.stdout = None
        self.stderr = None
        threading.Thread.__init__(self)

    def run(self, req):
    	p = Popen(['rsync'] + args + [req.source_path, req.destination_path],shell=False,stdout=subprocess.PIPE,stderr=subprocess.PIPE)
		self.stdout, self.stderr = p.communicate()

		return p.returncode > -1; #Return true on successful transfer

def create_rsync_thread(req):
	t = threading.Thread(req)
    t.daemon = True
    t.start()

def rsync_server():
    rospy.init_node('rysnc_server')
    s = rospy.Service('rsync', RsyncRequest, handle_add_two_ints)

    rospy.loginfo("Ready to sync files.")
    rospy.spin()

if __name__ == "__main__":
    rsync_server()