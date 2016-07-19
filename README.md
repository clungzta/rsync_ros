#rsync_ros 
[![Build Status](http://build.ros.org/buildStatus/icon?job=Jdev__rsync_ros__ubuntu_trusty_amd64&build=3)](http://build.ros.org/job/Jdev__rsync_ros__ubuntu_trusty_amd64/3/)

##Overview
This package brings Rsync operations (file transfer and syncronization) to ROS in the form of an [actionlib](http://wiki.ros.org/actionlib) interface. Making it convenient to transfer files from one machine to another using ROS. Progress from transfers is published as action feedback.

##Potential Use Cases
An RsyncAction goal could be sent once a robot has reached a certain state. A file (e.g. a logfile or bagfile) could be then be transferred to a remote fileserver for storage.

##Why Rsync?
[Rsync](http://linux.die.net/man/1/rsync) is a fast and versatile utllity for file transfer, it is widely used for backups and mirroring, as well as an improved copy command for everyday use. It can copy locally, to/from another host over any remote shell, offering a large number of options that control every aspect of its behavior. It is famous for its delta-transfer algorithm, which reduces the amount of data sent over the network by sending only the differences between the source files and the existing files in the destination.

##Usage

###To start the server
`rosrun rsync_ros rsync_server_node.py`

####CLI Actionlib Client Example
scripts/rsync_client_example.py, a simple client based on the actionlib_tutorials [simple actionlib client](http://wiki.ros.org/actionlib_tutorials/Tutorials/Writing%20a%20Simple%20Action%20Client%20%28Python%29).

#####Local transfer
`rosrun rsync_ros rsync_client_example.py -avzp ~/a_file.txt ~/file_dest.txt`

#####Remote transfer
You will first need to set up ssh keys for the remote machine, see this [tutorial](https://www.digitalocean.com/community/tutorials/how-to-set-up-ssh-keys--2)  
`rosrun rsync_ros rsync_client_example.py -avzp ~/a_file.txt ssh_user@remote_host:~/file_dest.txt`

##Choosing Rysnc Arguments
The server passes all arguments listed in rsync_args to Rsync, although not all of the Rsync arguments have been tested. If you're unfamiliar with RSync, have a look at the [manual](http://linux.die.net/man/1/rsync) to see the full list of arguments.

*TL;DR* The `-avz` argument works well for copy operations, and `-avz --delete-source-files` arguments for move operations. The `-p` argument (Partial Transfer) is particuarly useful for dealing with unreliable (wireless) connections.


##Simple Python Actionlib example  
``` python
#!/usr/bin/env python
import rospy
import roslib; roslib.load_manifest('rsync_ros')
import actionlib
import rsync_ros.msg
import sys

rospy.init_node('rsync_client')

source_path =  '~/a_file.txt'
dest_path = 'SSH_USER@192.168.0.1:~/file_dest.txt'

# Create the SimpleActionClient, passing RsyncAction to the constructor.
client = actionlib.SimpleActionClient('rsync_ros', rsync_ros.msg.RsyncAction)

# Wait until the action server has started up and started
# listening for goals.
client.wait_for_server()

# Create a goal to send to the action server.
goal = rsync_ros.msg.RsyncGoal(rsync_args=['-avzp'], source_path=source_path, destination_path=dest_path)

# Sends goal to the action server.
client.send_goal(goal)

# Waits for the server to finish performing the action.
client.wait_for_result()

# Gets the result of the action
result = client.get_result()

# Prints out the result of executing the action
rospy.loginfo("Successful Transfer: {}".format(result.sync_success))  # An RsyncResult
```

##Rsync Action Definition
```
#Goal
string[] rsync_args #List of Rsync command line arguments e.g. ['-avzh', '--partial']
                    
string source_path #e.g. "/home/user/file_to_sync.txt"
string destination_path #e.g. "ssh_username@192.168.0.1:/home/user/file_destination.txt"

---
#Result
bool sync_success #returns true if the file(s) synced correctly

---
#Feedback
float32 percent_complete
int64 transfer_rate #in bytes/second
```
