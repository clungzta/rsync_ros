#rsync_ros
##Overview
[Rsync](http://linux.die.net/man/1/rsync) is a utllity for file syncronisation and file transfer between multuple locations. Rsync is typically used to synchronize files and directories between two different systems over SSH, connecting as a user to a remote-host. See [https://en.wikipedia.org/wiki/Rsync](https://en.wikipedia.org/wiki/Rsync) for information.

This package brings Rsync functionallity to the ROS. With the intent of bringing file transfer (move, copy) and syncronisation operations in the form of an [actionlib](http://wiki.ros.org/actionlib) API. This API makes it convenient to transfer a file (e.g. a logfile or bagfile) from one machine to another.

##Potential Use Cases
An rsync action goal may be sent when a robot has reached a certain state (e.g. using [smach](http://wiki.ros.org/smach)) or when a sensor threshold is reached. A a file (e.g. a logfile or bagfile) containg data recorded prior to the event could be then tranfered to a fileserver for storage.

server_node.py is an ActionServer. The node calls Rsync through [subprocess Popen](https://docs.python.org/2/library/subprocess.html#popen-constructor) when it recives a ROS action goal. The actionServer provides feedback on the current transfer percentage and data throughput, returning a flag result upon completion of the transfer.

The server passes all arguments listed in rsync_args to Rsync. See the available arguments [here](http://linux.die.net/man/1/rsync).

##Requrements
* Rsync version 3.1.0+ (Developed and tested with this version, other versions are not currently supported due to the lack of the `--outbuf` flag.)
* ssh is required if you are transfering to and/or from remote machine(s)
* actionlib
* ROS

##Rsync Action Definition
```
#Goal
string[] rsync_args #List of Rsync command line arguments e.g. ['-avzh', '--partial']
                    #see http://linux.die.net/man/1/rsync
                    
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
