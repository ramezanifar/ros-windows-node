# ros-windows-node
How to have a ros node in Windows created by python to talk to roscore in Linux

### Problem statement:  
roscore is in Linux. We want to use the rospy library to create an application and run it in Windows so that this node can communicate with the roscore.

### Network assumptions  
- Linux machine IP address is 192.168.1.10  
- Windows machine IP address is 192.168.1.20  
- You can ping a machine from other one

### Steps  
1- Copy the content of the Linux directory of **/opt/ros/melodic/lib/python2.7/dist-packages** and put it in a desired folder in Windows. For example **D:\Test\ros_packages**

2- Copy the the Linux directory of **/opt/ros/melodic/share/rosgraph** and put it in **D:\Test**  

3- Assuming you have python2.7 and pip installed in Windows. Go to the path **C:\Python27\Lib\site-packages** and screenshot the existing files and folders. Next we want to install some packages and move them outside. So we need to know the differences.

4- Install yaml. Open a cmd and run:  
pip install pyYaml

5- Install rospkg.  Open a cmd and run:  
pip install rospkg

6- Cut all the added folders from **C:\Python27\Lib\site-packages** to the same folder where you put ros stuff. In our example **D:\Test\ros_packages**

7- Create a python file in **D:\Test\config.py** and put the following code in it:
```python
import os, sys  
os.environ['ROS_MASTER_URI'] = 'http://192.168.1.30:11311'  #  This is to find ros master
os.environ['ROS_IP'] = '192.168.1.20'  #  This is used as the identity of the node in ros network
os.environ['ROS_PACKAGE_PATH'] = os.path.abspath(".")  #  This is to ensure rosgraph is known
sys.path.insert(0, "./ros_packages")  #  This is to ensure rospy, rosnode , etc packages are known
```
  
8- Put your main python code in  **D:\Test**   
9- In your main python code import config.py   
10- Set the environemnt variables in Linux before running any ros command such as roscore, rostopic echo , etc. :

source /opt/ros/melodic/setup.bash   
export ROS_MASTER_URI=http://192.168.1.10:11311   
export ROS_IP=192.168.1.10   

These lines can be added in the .bashrc and as a result when you open a terminal they will be executed.  
11- Run roscore and your code  

### Sample code for Listener
Consider the simple Talker/Listener example. Suppose the takher is in Linux and Listener is in Windows. We modified the standar Listener code to cope with the roscore statuses. When we run the Listener, roscore might not be up. Also after connection was stablished, roscore may go down any time. So it is important for Listener to check on roscore frequently. here is a sample:
```python
import config_ros_win  # Load all environment variables

import rospy, rosgraph
from std_msgs.msg import String
import time
import threading


class Listener:
    ''' This class is a listener to a topic called /chatter of type string published by talker node from Linux
    where the master is. The goal is to accurately track the status of the roscore and respond appropriately.
     '''
    def __init__(self):
        self.run = True  # flag to run the program
        self.ros_state = 'greet'  # state machine to keep track of ros connection
        self.ros_thread = threading.Thread(target=self.ros_check_connect, args=())  # thread to check ROS  
        self.ros_thread.start()  # start the thread

    def ros_check_connect(self):
        """ This method is used to check the ROS connection """
        while self.run is True:
            # Run the state machine
            if self.ros_state == 'greet':
                print "Attempting to connect to roscore. Please wait ... "
                self.ros_state = 'init'
            elif self.ros_state == 'init':  # state to connect to ros master 
                if rosgraph.is_master_online() is True:  # Is ros master up and running?
                    rospy.init_node('Trend', anonymous=False, disable_signals=True)  # Start the node
                    rospy.Subscriber('chatter', String, self.callback)  # subscribe to a topic called /chatter
                    self.ros_state = 'online'  # Change the state
                    print "Listener node was created"

            elif self.ros_state == 'online':  # roscore is online and we are connected to it
                if rosgraph.is_master_online() is False:  # Every one second check the ros master
                    self.ros_state = 'dead'  # roscore went down after this listener established connection to it

            elif self.ros_state == 'dead':  # roscore went down after we connected to it.   
                # rospy cannot reconnect to roscore and it has to restart  
                print "roscore is dead. Application is terminated"
                self.run = False  # Terminate the while loop

            time.sleep(1)  #  Check the ros connection at slow rate

    def callback(self,data):
        # Callback for the subscriber
        print data.data



if __name__ == '__main__':
    my_listener = Listener()  # Create an object of the Listener class
    # Run the main thread. this is to capture ctrl+c
    while my_listener.run == True:
        try:
            time.sleep(1)  # Just idle until ctrl+c is detected
        except KeyboardInterrupt:
            print "ctrl+c was detected. Listener node is terminated"
            my_listener.run = False  # this will terminate the thread used for ros checking

```

### Troubleshooting
1- In Linux make sure the node created in Windows appears in rosnode list.   
2- use roswtf to see if there is any error  
3- Check your Windows and Linux firewall

