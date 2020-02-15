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

2- Copy the content of the Linux directory of **/opt/ros/melodic/share/rosgraph** and put it in **D:\Test**
2- Assuming you have python2.7 and pip installed in Windows. Go to the path **C:\Python27\Lib\site-packages** and screenshot the existing files and folders. Next we want to install some packages and move them outside. So we need to know the differences.

3- Install yaml. Open a cmd and run:  
pip install pyYaml

4- Install rospkg.  Open a cmd and run:  
pip install rospkg

5- Cut all the added folders from **C:\Python27\Lib\site-packages** to the same folder where you put ros stuff. In our example **D:\Test\ros_packages**

6- Create a python file in **D:\Test\config.py** and put the following code in it:
```python
import os, sys  
os.environ['ROS_MASTER_URI'] = 'http://192.168.1.30:11311'  
os.environ['ROS_IP'] = '192.168.1.20'
os.environ['ROS_PACKAGE_PATH'] = os.path.abspath(".")  #  This is to ensure rosgraph is known
sys.path.insert(0, "./ros_packages")  #  This is to ensure rospy, rosnode , etc packages are known
```
  
7- Put your main python code in  **D:\Test**   
8- In your main python code import config.py   
9- Set the environemnt variables in Linux before running any ros command such as roscore, rostopic echo , etc. :

source /opt/ros/melodic/setup.bash   
export ROS_MASTER_URI=http://192.168.1.10:11311   
export ROS_IP=192.168.1.10   

These lines can be added in the .bashrc and as a result when you open a terminal they will be executed.  
10- Run roscore and your code  

### Troubleshooting
1- In Linux make sure the node created in Windows appears in rosnode list.   
2- use roswtf to see if there is any error  
3- Check your Windows and Linux firewall

