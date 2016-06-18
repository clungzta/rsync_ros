#rsync_actionlib
##Description
A ROS [actionlib](http://wiki.ros.org/actionlib) server for [Rsync](https://en.wikipedia.org/wiki/Rsync).

[Rsync](https://en.wikipedia.org/wiki/Rsync) is the de-facto standard on unix-like systems for syncing files and folders from one location to another (local machine or a remote machine through ssh).

rsync_node.py allows a user to call Rsync through the ROS action protocol. It returns a flag result upon completion of the transfer.

[Rsync](https://en.wikipedia.org/wiki/Rsync) is called through a python subprocess.

Passes through Rsync command line arguments, entered in a list form to rsync_args. See the available arguments [here](http://linux.die.net/man/1/rsync).

##Use Cases
Automated file transfers (logfiles, rosbags, image files) between robot(s) and other machines.

##Requirements

###Server
ROS, Rsync, actionlib, ssh and python must be installed.

###Rsync client machine
ssh and Rsync must be installed

##Rsync Action Definition
```
#Goal
string[] rsync_args #List of Rsync command line arguments e.g. ['-avzh', '--partial']
                    #see http://linux.die.net/man/1/rsync
                    
string source_path #e.g. "/home/user/folder_to_sync", "/home/user/file_to_sync.txt"
string destination_path #e.g. "remote_user@192.168.0.1:/home/user/folder_destination", "/home/user/file_destination.txt"

---
#Result
bool sync_success #returns True if the file(s)/folder(s) synced correctly

---
#Feedback
float32 percent_complete
```
