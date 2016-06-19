#rsync_actionlib
##Description
A ROS [actionlib](http://wiki.ros.org/actionlib) server for [Rsync](https://en.wikipedia.org/wiki/Rsync).

[Rsync](https://en.wikipedia.org/wiki/Rsync) is the de-facto standard on unix-like systems for syncing files and folders from one location to another (either the local machine or a remote machine through ssh).

server_node.py is an ActionServer. The ActionServer node calls Rsync through [subprocess Popen](https://docs.python.org/2/library/subprocess.html#popen-constructor) when it recives a ROS action goal. The ActionServer returns a flag result upon completion of the transfer.

The server passes all arguments listed in rsync_args to Rsync. See the available arguments [here](http://linux.die.net/man/1/rsync).

##Requrements
* **Rsync version 3.1.0+** (Developed and tested with this version, other versions are not currently supported due to the lack of the `--outbuf` flag.)
* **ssh** is required if you are transfering to and/or from remote machine(s)
* **actionlib**
* **ROS**

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
```
