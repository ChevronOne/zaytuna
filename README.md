<p align="center">
    <img width="400" align="center" src="tex/1280px-Basmala" alt="Basmala" />
</p>

<br/><br/>

# Self-Driving Car Simulator with ROS  :deciduous_tree:
Zaytuna is a very simple, lightweight self-driving car simulator that is programmable with [ROS](https://www.ros.org).


**Prerequirements:**  [ROS](https://www.ros.org).  
If you haven't yet, please install a [ROS](https://www.ros.org) version that is supported on your operating system. And make sure that ROS [environment variables](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment#Managing_Your_Environment) are setup properly.
<br/><br/>
**Building the package:** If you know how to build a [ROS package](http://wiki.ros.org/ROS/Tutorials/CreatingPackage), you don't need to read the rest. Otherwise, it is more convenient to go through these [tutorials](http://wiki.ros.org/ROS/Tutorials) first.  
To build tha package navigate to `src` folder in your [catkin workspace](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment#Create_a_ROS_Workspace), assuming your catkin workspace is named `catkin_ws`, then run the following commands:
+ `:~/catkin_ws/src$ git clone https://github.com/ChevronOne/zaytuna.git`
+ `:~/catkin_ws/src$ cd ..`
+ `:~/catkin_ws$ catkin_make`

If the build went successfully, open a second terminal and run the command `roscore`, and in the current terminal run the command `rosrun zaytuna zay_simu`. And there you have it!

To see the topics of the simulated model vehicle, open a third terminal and run the command `rostopic list`. Currently there is no documentation or tutorials of how to use them, but if you're familiar with ROS, then most of them should be self-explanatory.  
To navigate in the scene use `W`, `S`, `A`, `D`, `Up`, `Down`, `Right` and `Left` keys. And for orientation use the mouse pointer. If a model vehicle is connected to control panel with *Key-Controller* on, you can move it with `T`, `G`, `F` and `H` keys.

The simulator provides simulations for almost all basic sensors that are needed for a self-driving car to be programmed. The only topic that's missing is **Lidar**, but the model vehicle can be fully driven by vision.  
The subscriber of *speed* and *steering* each takes a value between [-1,1]. For steering value 1.0 simulates 25 degrees right-turn of front wheels, and -1.0 the same for left turn. And for speed 1.0 simulates full speed in forward direction and -1.0 for backward.

Depending on how much it could be used, there might be a consideration to work on a lidar-simulation as well. The aim is to keep it simple and lightweight as possible, and avoid using any heavy dependencies or physic libraries.  
<br/><br/>
**Note:** The word Zaytuna means '*an olive tree*' :seedling: