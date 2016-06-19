# Software License Agreement (BSD License)
#
# Copyright (c) 2016, Alex McClung
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

#!/usr/bin/env python
import rospy
import roslib
roslib.load_manifest('rsync_actionlib')
import actionlib
import re
from subprocess import Popen, PIPE
from rsync_ros.msg import RsyncAction, RsyncResult, RsyncFeedback

class Rsync():

    def __init__(self, rsync_args, source, dest, progress_update_callback=None):
        self.rsync_args = rsync_args
        self.source = source
        self.dest = dest
        self.percent_complete = 0
        self.progress_update_callback = progress_update_callback

        self.stdout_block = ''
        self.stderr_block = ''
    
    def sync(self):
        #Sync the files
        rsync_cmd = ['rsync'] + self.rsync_args + ['--progress', '--outbuf=L', self.source, self.dest]
        rospy.loginfo("Executing rsync command '%s'", ' '.join(rsync_cmd))
        self.p = Popen(rsync_cmd, stdout=PIPE, stderr=PIPE)
        
        #Catch stdout from RSync in (near) real-time
        for line in iter(self.p.stdout.readline, b''):
            rospy.loginfo(line)
            self.stdout_block += line

            #Calculate percentage by parsing the stdout line
            if self.progress_update_callback:
                self._update_progress(line)
                self.progress_update_callback(self.percent_complete)

        self.stderr_block = '\n'.join(self.p.stderr)

        self.p.poll()

        if self.p.returncode > -1:
            #Set feedback to 100% complete, for cases when no progress is piped from Rsync stdout
            self.progress_update_callback(100.0)
            return True
        else:
            return False

    '''
    def calculate_total_files(self):
    #Dry Run, Calculates the Total Number of files to be transfered, not syncing        
    p = Popen(['rsync', '-az', '--stats', '--dry-run', self.source, self.dest], stdin=PIPE,  stdout=PIPE)
    self.total_files = int(re.findall(r'Number of files: (\d+)', p.communicate()[0])[0])
    return self.total_files
    '''

    def _update_progress(self, stdout_line):
        #Calculate Sync Progress by parsing a line of stdout
        if 'to-chk' in stdout_line:
            m = re.findall(r'to-chk=(\d+)/(\d+)', stdout_line)
            self.total_files = float(m[0][1])
            self.remaining_files = float(m[0][0])

            self.percent_complete = 100.0 * (1 - (self.remaining_files/self.total_files))

    def get_progress(self):
        return self.percent_complete

class RsyncActionServer:

    def __init__(self, name):
        self._action_name = name
        self.server = actionlib.SimpleActionServer(self._action_name, RsyncAction, self.execute, False)
        self.server.start()
        rospy.loginfo("Ready to sync files.")

    def progress_update_cb(self, percent_complete):
        #This is run everytime the progress is published to stdout
        #rospy.loginfo('Total transfer percentage: {}'.format(percent_complete))
        self.feedback.percent_complete = percent_complete
        self.server.publish_feedback(self.feedback)

        # check if preempt (cancel action) has been requested by the client
        if self.server.is_preempt_requested():
            rospy.loginfo('%s: Preempted' % self._action_name)
            self.rsync = None #Delete the instance of the Rsync class
            self.server.set_preempted() #TO-DO, fix logic error changing states upon preempt request

    def execute(self, goal):
        self.result = RsyncResult()
        self.feedback = RsyncFeedback()

        self.rsync = Rsync(goal.rsync_args, goal.source_path, goal.destination_path, progress_update_callback=self.progress_update_cb)

        self.result.sync_success = self.rsync.sync()

        if not self.server.is_preempt_requested():

            if self.rsync.stderr_block:
                rospy.logerr('\n{}'.format(self.rsync.stderr_block))

            rospy.loginfo("Rsync command result '%s'", self.result.sync_success)
            self.server.set_succeeded(self.result)

if __name__ == "__main__":
    try:
        rospy.init_node('rsync_actionlib')
        RsyncActionServer(rospy.get_name())
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
