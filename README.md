#rsync_ros
##Description
A ROS [actionlib](http://wiki.ros.org/actionlib) server for [Rsync](https://en.wikipedia.org/wiki/Rsync).

Copy or Sync files and/or folders from one location to another. 
The location can be either on the local machine or a remote machine through ssh.

A Rsync python subprocess is called from a ROS action goal.
[Rsync](https://en.wikipedia.org/wiki/Rsync) is called through a python subprocess.

Supports the full list of Rsync command line arguments, http://linux.die.net/man/1/rsync

##Requirements

To use this script your system needs to have ROS, rsync, ssh and python installed.

##Rsync Action Definition
```
#Goal
string[] rsync_args #List of Rsync command line arguments e.g. ['-avzh', '--partial'], see http://linux.die.net/man/1/rsync
string source_path #e.g. "/home/user/folder_to_sync", "/home/user/file_to_sync.txt"
string destination_path #e.g. "remote_username@192.168.0.1:/home/user/folder_to_sync", "/home/user/file_to_sync.txt"

---
#Result
bool sync_success #returns True if the file(s)/[folder(s) synced correctly

---
#Feedback
float32 percent_complete
```