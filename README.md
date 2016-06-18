#rsync_actionlib
##Description
A ROS [actionlib](http://wiki.ros.org/actionlib) server for [Rsync](https://en.wikipedia.org/wiki/Rsync).

[Rsync](https://en.wikipedia.org/wiki/Rsync) is the de-facto standard on unix-like systems for syncing files and folders from one location to another (local machine or a remote machine through ssh).

rsync_node.py allows a user to call Rsync through the ROS action protocol. It returns a flag result upon completion of the transfer. The node calls Rsync through a subprocess call.

The server passes all arguments listed in rsync_args to Rsync. See the available arguments [here](http://linux.die.net/man/1/rsync).

##Dependencies

####On Actionlib/Rsync Server
Rsync, actionlib, ssh, rospy

####Rsync client machine
ssh and Rsync

##Rsync Action Definition
```
#Goal
string[] rsync_args #List of Rsync command line arguments e.g. ['-avzh', '--partial']
                    #see http://linux.die.net/man/1/rsync
                    
string source_path #e.g. "/home/user/file_to_sync.txt"
string destination_path #e.g. "ssh_username@192.168.0.1:/home/user/file_destination.txt"

---
#Result
bool sync_success #returns True if the file(s)/folder(s) synced correctly

---
#Feedback
float32 percent_complete
```
