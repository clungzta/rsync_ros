#rsync_ros
##Overview
[Rsync](http://linux.die.net/man/1/rsync) is a utllity for file syncronisation and file transfer between multuple locations. Rsync is typically used to synchronize files and directories between two different systems over SSH, connecting as a user to a remote-host. See [https://en.wikipedia.org/wiki/Rsync](https://en.wikipedia.org/wiki/Rsync) for information.

This package brings Rsync functionality to the ROS. With the intent of bringing file transfer (move, copy) and syncronisation operations in the form of an [actionlib](http://wiki.ros.org/actionlib) API. This API makes it convenient to transfer a file from one machine to another.

server_node.py is an ActionServer. The node calls Rsync through [subprocess Popen](https://docs.python.org/2/library/subprocess.html#popen-constructor) when it receives a ROS action goal. The action Server provides feedback on the current transfer percentage and data throughput, returning a flag result upon completion of the transfer.

The server passes all arguments listed in rsync_args to Rsync. See the available arguments [here](http://linux.die.net/man/1/rsync).

##Potential Use Cases
A rsyncAction goal could be sent when a robot has reached a certain state (e.g. using [smach](http://wiki.ros.org/smach)) or when a sensor threshold has been reached. A a file (e.g. a logfile, map, image snapshot or bagfile) could be then transferred to a remote fileserver for storage.

##Requirements
* Rsync version 3.1.0+ (Developed and tested with this version, other versions are not currently supported due to the lack of the `--outbuf` flag.)
* ssh is required if you are transferring to and/or from remote machine(s)
* actionlib
* ROS

##Example
Have a look at scripts/rsync_client_example.py, it is a simple client based on the actionlib_tutorials [simple actionlib client](http://wiki.ros.org/actionlib_tutorials/Tutorials/Writing%20a%20Simple%20Action%20Client%20%28Python%29).

####Start the server
`rosrun rsync_ros rsync_server.py`

####Run the Client
#####Local transfer example
`rosrun rsync_ros rsync_client_example.py -avz ~/a_file.txt ~/copied_file.txt`

#####Remote transfer example
You will first need to set up ssh keys for the remote machine, see this [tutorial](https://www.digitalocean.com/community/tutorials/how-to-set-up-ssh-keys--2)  
`rosrun rsync_ros rsync_client_example.py -avzP ~/a_file.txt ssh_username@192.168.0.1:~/copied_file.txt`

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
