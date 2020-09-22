<p align="center">
    <img width="400" align="center" src="tex/Calligraphy" alt="Calligraphy" />
</p>

<br/><br/>

# Self-Driving Car Simulator with ROS  :deciduous_tree:
**Zaytuna** is a very simple, lightweight self-driving car simulator that is programmable with [ROS](https://www.ros.org).  
<br/><br/>
:exclamation: **Note:** Not suitable for virtual machine. You'll not be able to run it in Virtualbox or on a guest operating system. And if you've ever got it to run on a guest OS, the scene won't render properly, and therefore will not be suitable to drive by vision. For a simulator you usually need an actual graphic card not emulated one.  

**Requirements:**   ROS.  "*No support for ROS2*"  
If you haven't yet, please install a [ROS](https://www.ros.org) version that is supported on your operating system. And make sure that ROS [environment variables](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment#Managing_Your_Environment) are setup properly.
<br/><br/>
**Building the package:** If you know how to build a [ROS package](http://wiki.ros.org/ROS/Tutorials/CreatingPackage), you probably don't need to read the rest. Otherwise, it's more convenient to go through these [tutorials](http://wiki.ros.org/ROS/Tutorials) first.  
To build the package open a terminal and navigate to `src` folder in your [catkin workspace](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment#Create_a_ROS_Workspace), assuming your catkin workspace is named `catkin_ws`, then run the following commands:  
+ `:~/catkin_ws/src$ git clone https://github.com/ChevronOne/zaytuna.git`
+ `:~/catkin_ws/src$ cd ..`
+ `:~/catkin_ws$ catkin_make`

If the build was successful, open a second terminal and run the command `roscore`, and in the current terminal run the command `rosrun zaytuna zay_simu`. And there you have it!

To see the topics of the simulated model vehicle, open a third terminal and run the command `rostopic list`. Currently there is no documentation or tutorials of how to use them, but if you're familiar with ROS, then most of them should be self-explanatory.  
To navigate in the scene use `W`, `S`, `A`, `D`, `Up`, `Down`, `Right` and `Left` keys. And for orientation use mouse pointer. If a model vehicle is connected to control panel with *Key-Controller* chosen, you can move it with `T`, `G`, `F` and `H` keys.

The simulator provides simulations for almost all basic sensors that are needed for a self-driving car to be programmed. The only topic that's missing is **LiDAR**/**Laser Scanner**, but a model vehicle still can be fully driven by vision.  
The subscribers of *speed* and *steering* each takes a value within a range of [-1,1]. For steering value 1.0 simulates 25 degrees right-turn of front wheels, and -1.0 the same for left turn. And for speed 1.0 simulates full speed in forward direction and -1.0 the same for backward direction.

Depending on how much it could be used, there might be a consideration to work on a lidar-simulation as well. However the aim is to keep it simple and lightweight as possible, and avoid using any heavy dependencies or physic libraries.  
<br/><br/>
**Note:** The word Zaytuna means '*an olive tree*' :seedling: