#rsync_ros
##Overview
This package brings Rsync functionality to the ROS. With the intent of bringing file transfer (move, copy) and syncronisation operations in the form of an [actionlib](http://wiki.ros.org/actionlib) interface. The interface makes it convenient and to transfer a file from one machine to another using ROS, progress from the transfer is published as action feedback.

server_node.py is an ActionServer. The node calls Rsync as a subprocess when it receives a ROS action goal. The node parses the current transfer percentage and data throughput from stdout and publishes the parsed data as action feedback. Once the transfer has completed the node returns a `sync_success` action result.

##Potential Use Cases
An RsyncAction goal could be sent when a robot has reached a certain state (e.g. using [smach](http://wiki.ros.org/smach)) or when a sensor threshold has been reached. A a file (e.g. a logfile or bagfile) could be then transferred to a remote fileserver for storage.

##Why Rsync?
[Rsync](http://linux.die.net/man/1/rsync) is a fast and versatile utllity for file transfer, it is widely used for backups and mirroring, as well as an improved copy command for everyday use. It can copy locally, to/from another host over any remote shell, it offers a large number of options that control every aspect of its behavior. It is famous for its delta-transfer algorithm, which reduces the amount of data sent over the network by sending only the differences between the source files and the existing files in the destination.

##Why actionlib?
See [http://wiki.ros.org/ROS/Patterns/Communication](http://wiki.ros.org/ROS/Patterns/Communication#Communication_via_Topics_vs_Services_vs_X)

##Requirements
* Rsync version 3.1.0+ (Developed and tested with this version, other versions are not currently supported due to the lack of the `--outbuf` flag.)
* ssh is required if you are transferring to/from remote machines
* actionlib

##Example
###Example Script
scripts/rsync_client_example.py, a simple client based on the actionlib_tutorials [simple actionlib client](http://wiki.ros.org/actionlib_tutorials/Tutorials/Writing%20a%20Simple%20Action%20Client%20%28Python%29).

###To Run Example
####Start the server
`rosrun rsync_ros rsync_server.py`

####Run the Client
#####Local transfer example
`rosrun rsync_ros rsync_client_example.py -avzp ~/a_file.txt ~/file_dest.txt`

#####Remote transfer example
You will first need to set up ssh keys for the remote machine, see this [tutorial](https://www.digitalocean.com/community/tutorials/how-to-set-up-ssh-keys--2)  
`rosrun rsync_ros rsync_client_example.py -avzp ~/a_file.txt ssh_user@remote_host:~/file_dest.txt`

##Rysnc Arguments
The server passes all arguments listed in rsync_args to Rsync, although not all of the Rsync arguments have been tested. If you're unfamiliar with RSync, have a look at the [manual](http://linux.die.net/man/1/rsync) to see the full list of arguments.

*TL;DR* The `-avz` argument works well for copy operations, and `-avz --delete-source-files` arguments for move operations. The `-p` argument (Partial Transfer) is particuarly useful for dealing with unreliable (wireless) connections.


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
