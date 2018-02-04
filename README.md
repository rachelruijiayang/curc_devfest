
<a id="top"></a>
# CURC DevFest Robotics Track
*How to build robots using Robot Opeating System, OpenCV, and other ROS packages.*

Written and developed by [Rachel Yang](https://github.com/rachelruijiayang), [Ignacio Ramirez-Piriz](https://github.com/ir2331), and [Columbia University Robotics Club](http://www.columbia.edu/cu/roboticsclub/). 

Portions of this tutorial are adapted from [NVIDIA's Robotics Teaching Kit with 'Jet'](https://developer.nvidia.com/teaching-kits), [*ROS Robotics Projects* by Lentin Joseph](https://www.amazon.com/ROS-Robotics-Projects-Lentin-Joseph/dp/1783554711) (available to Columbia students through [CLIO](https://clio.columbia.edu/quicksearch?q=ros+robotics+projects)), and [*Effective Robotics Programming with ROS - Third Edition* by Anil Mahtani and Luis Sanchez](https://www.amazon.com/Effective-Robotics-Programming-ROS-Third/dp/1786463652) (also available through [CLIO](https://clio.columbia.edu/quicksearch?q=effective+robotics+programming+with+ros+third+edition&commit=Search)).

## What We're Building
Throughout this tutorial, we're going to build a ROS system that uses a webcam and computer vision to *sense* the location of the user's face and then *actuate* a simulated turtle to follow the user's face.

This document advances linearly, meaning that level 2 builds atop level 1, level 3 atop level 2, and so on. Each level after level 1 has its own branch in the [GitHub repository](https://github.com/rachelruijiayang/catkin_ws) for this tutorial. If you don't want to do each level in order, you can `git checkout` the branch for the level you would like to skip to.

## Why learn ROS?
ROS is an open-source framework for building robots. 

A robot is just an system in which sensors (such as cameras) connect to processors, which connect to actuators to make the robot move (or take some other action). ROS provides a publish-subscribe messaging infrastructure that makes it easier for different parts of a robot system to talk to each other. ROS also provides an extensive set of tools for configuring, starting, debugging, simulating, testing, and stopping robot systems. Finally, ROS provides a broad collection of open-source libraries that implement useful robot functionality, with a focus on mobility, manipulation, and perception, and a large active developer community. [[Source]](https://answers.ros.org/question/12230/what-is-ros-exactly-middleware-framework-operating-system/)

[Here are some past robots that have been built using ROS.](https://www.youtube.com/watch?v=Z70_3wMFO24)

## Prerequisites

 - A computer with a webcam (required for the computer vision sections)
 - At least 25 GB of free storage space on your computer (to install the VM)
 - Prior programming experience, especially in C++
 - Familiarity with using the Linux terminal

# Table of Contents
- [Level 0: Environment Setup](#level0)
- [Level 1: Intro to ROS](#level1)
- [Level 2: Basic OpenCV](#level2)
- [Level 3: Using CV Output to Actuate the Turtle](#level3)
- [Level 4: Face Detection using OpenCV](#level4)
- [Level 5: What's next?](#level5)

<a href="#top" class="top" id="level0"></a>
# Level 0: Environment Setup
To develop in the [most recent version of ROS](http://wiki.ros.org/kinetic), you will need to be running Ubuntu 16.04 as your OS. We will be using [VirtualBox](https://www.virtualbox.org/manual/ch01.html) to run a custom Ubuntu 16.04 VM image that already has ROS and the other dependencies for this tutorial installed on it.

## Setting Up VirtualBox
Download VirtualBox from this page: https://www.virtualbox.org/wiki/Downloads

Under **VirtualBox 5.2.6 platform packages**, select your host OS, and then download and click on the executable. Click "Next" or "Install" at each prompt.

Next, from the same page, download **VirtualBox 5.2.6 Oracle VM VirtualBox Extension Pack**. Click on the link "All supported platforms". You will start downloading a file named **Oracle_VM_VirtualBox_Extension_Pack-5.2.6-120293.vbox-extpack**. Click on the file, and after the VirtualBox window pops up, click **Install**.
![enter image description here](https://github.com/rachelruijiayang/curc_devfest/blob/master/resources/vmware_extension_pack.PNG?raw=true)

### Importing the RoboticsTrack VM Image
Download **RoboticsTrack.ova** from [this link](https://drive.google.com/file/d/1ctli6AgX2t4oW2emczZkN71z1-X41zfa/view?usp=sharing). An .ova file describes a virtual machine.
Then, inside Oracle VM VirtualBox Manager, select File > Import Appliance, click on the folder icon, and find **RoboticsTrack.ova**. 

Review the appliance settings. Because my computer has 12 GB of RAM, I selected 8192 MB (8 GB) of RAM as the default amount for **RoboticsTrack**.
**If your computer has less than 12 GB of RAM** (Here's how [Mac](http://www.macinstruct.com/node/422) users find out how much RAM they have; here's how [Windows](https://www.computerhope.com/issues/ch000149.htm) users find out.): Lower the amount of RAM used to 2/3 of your computer's physical RAM.  Adjust the value by double clicking on 8192 MB and typing in your new value. (1 GB = 1000 MB).

Finally, click "Import". Congratulations, you have set up your ROS development environment!

Start up and log into your Ubuntu 16.04 VM using these credentials:
Username: `robot`
Password: `robot`

Open up the terminal (Ctrl-Alt-t), `cd` into the home directory of `robot`, and run
```
$ git clone https://github.com/rachelruijiayang/catkin_ws.git
```
to download the workspace in which you will be writing and building your code. Then, at the bottom of `~/.bashrc`, add the line:
```
source ~/catkin_ws/devel/setup.bash
```
to add our `catkin_ws` into the PATH that ROS uses to search for executables. Finally, run
```
$ source ~/.bashrc
```
to reload your `bash` settings.

<a href="#top" class="top" id="level1"></a>
# Level 1: Intro to ROS
In this level, you will learn the structure and fundamental building blocks of ROS. Then, using those concepts, you'll write some C++ ROS code to programmatically control a simulated turtle. 

ROS has three levels of concepts: the Filesystem level, the Computation Graph level, and the Community level. These levels and the most important components of them are summarized below. See http://wiki.ros.org/ROS/Concepts for the full list.

## The ROS Filesystem Level
The filesystem level concepts mainly cover ROS resources that you encounter on disk, such as:
- **Workspaces**: A workspace is a folder where you modify, build, and install ROS packages (also called catkin packages). 
	- `~/catkin_ws` is the workspace for many of the ROS packages we will write and build in this tutorial.
- **Packages**: Packages are the main unit for organizing software in ROS. Generally, each package implements some particular functionality that can be integrated into a larger system. A package may contain ROS runtime processes (nodes), a ROS-dependent library, datasets, configuration files, or anything else that is usefully organized together. The most granular thing you can build and release is a package.
	- If you `cd` into `~/catkin_ws/src`, you will see a directory called `usb_cam`. `usb_cam` is a ROS package (documentation: http://wiki.ros.org/usb_cam). The functionality of `usb_cam` is to interface with standard USB cameras (including webcams) and convert a raw image stream from the camera into a stream of ROS image messages.
- **Package Manifests**: In the root of each ROS package, there is a file called package.xml, which is the package manifest. Manifests provide metadata about a package, including its name, version, description, license information, dependencies, and other meta information like exported packages.
	- Check out the manifest for `usb_cam` at `~/catkin_ws/src/usb_cam/package.xml`
- **Message (msg) types**: Message descriptions, stored in {package_name}/msg/{my_message_type}.msg, define the data structures for messages sent in ROS.
	- `usb_cam` has a single node (exectuable) that converts images from the USB camera into ROS messages of type `sensor_msgs/Image`. [Documentation for `sensor_msgs/Image`](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html)

## The ROS Computation Graph
A robot system can be thought of as a peer-to-peer network of sensors, processors, and actuators. Each of these components will run a ROS process. The ROS Computation Graph is the peer-to-peer network of ROS processes that make up one robot system and are processing data together. The basic Computation Graph concepts of ROS are nodes, Master, Parameter Server, messages, and topics.

![ROS Computation Graph](http://ros.org/images/wiki/ROS_basic_concepts.png)
- **Nodes**: Nodes are processes that perform computation. ROS is designed to be modular at a fine-grained scale; a robot control system usually comprises many nodes. For example, one node controls a laser range-finder, one node controls the wheel motors, one node performs localization, one node performs path planning, one Node provides a graphical view of the system, and so on. A ROS node is written with the use of a ROS client library, such as roscpp or rospy.
- **Master**: The ROS Master provides name registration and lookup to the rest of the Computation Graph. Without the Master, nodes would not be able to find each other, exchange messages, or invoke services.
- **Parameter Server**: The Parameter Server allows data to be stored by key in a central location. It is currently part of the Master.
- **Messages**: Nodes communicate with each other by passing messages. A message is simply a data structure, comprising typed fields. Standard primitive types (integer, floating point, boolean, etc.) are supported, as are arrays of primitive types. Messages can include arbitrarily nested structures and arrays (much like C structs).
- **Topics**: Messages are routed via a transport system with publish / subscribe semantics. A node sends out a message by *publishing* it to a given topic. The topic is a name that is used to identify the content of the message. A node that is interested in a certain kind of data will *subscribe* to the appropriate topic. There may be multiple concurrent *publishers* and *subscribers* for a single topic, and a single node may publish and/or subscribe to multiple topics. In general, publishers and subscribers are not aware of each others' existence. The idea is to *decouple the production of information from its consumption*. Logically, one can think of a topic as a strongly typed message bus. Each bus has a name, and anyone can connect to the bus to send or receive messages as long as they are the right type.

## Practice ROS Using Turtlesim
`turtlesim` is a ROS package that implements a simulated robot turtle. The turtle's position on the screen will change based on input velocity commands. We will see the above ROS concepts in action by using `turtlesim` as an example.
[[Source]](http://wiki.ros.org/ROS/Tutorials/UnderstandingNodes)

### Navigating through the ROS filesystem
If you want to find the path of the `turtlesim` package, you can use the following command:
```
$ rospack find turtlesim
```
Which will then result in the following output:
```
/opt/ros/kinetic/share/turtlesim
```
To enter the directory of the `turtlesim` package, use:
```
$ roscd turtlesim
```
Your new current path will be `/opt/ros/kinetic/share/turtlesim`. 

### Playing with ROS Nodes
Open a new terminal and start `roscore` using the following command:
```
$ roscore
```
The command `roscore` starts a collection of nodes and programs that are pre-requisites of a ROS-based system. You must have a `roscore` running in order for ROS nodes to communicate. `roscore` will start up:
- a ROS Master
- a ROS Parameter Server
- a rosout logging node

Now that `roscore` is running, we are going to use `rosnode list` to list all the nodes running. Open up a new terminal (while keeping the terminal running `roscore` alive) and run:
```
$ rosnode list
```
- You will see:
	```
	/rosout
	```
This showed us that there is only one node running: `rosout`. `rosout` is always running as it collects and logs nodes' debugging output.

### Using `rosrun`
`rosrun` allows you to use the package name to directly run a node within a package (without having to know the package path).
Usage:
```
$ rosrun [package_name] [node_name]
```
To run the node from the `turtlesim` package named `turtlesim_node`, type:
```
$ rosrun turtlesim turtlesim_node
```
You will see the `turtlesim` window:![enter image description here](http://wiki.ros.org/ROS/Tutorials/UnderstandingNodes?action=AttachFile&do=get&target=turtlesim.png)

In a new terminal:
```
$ rosnode list
```
`/turtlesim` has been added to the list of running nodes:
```
/rosout
/turtlesim
```

### Understanding Topics
Without closing the previous terminals, you can drive the turtle around its window by running in a new terminal:
```
rosrun turtlesim turtle_telop_key
```
Now you can use the arrow (up, down, left, right) keys of the keyboard to drive the turtle around. If you can not drive the turtle **select the terminal window in which you started `turtle_teleop_key`** to make sure that the keys that you type are recorded.
![enter image description here](http://wiki.ros.org/ROS/Tutorials/UnderstandingTopics?action=AttachFile&do=get&target=turtle_key.png)

`turtle_teleop_key` converts input keystrokes into velocity commands for the turtle. For example, the up key is converted into a command to move with some velocity in the up direction.  `turtle_teleop_key` **publishes** those velocity commands on a ROS **topic**. Our original `turtlesim` node **subscribes** to the same topic to receive the velocity commands.

If you run `rosnode list` again in a new terminal, you should see that these three nodes are running:
```
/rosout
/turtlesim
/teleop_turtle
```

You can also use a tool called `rqt_graph` to graphically show the nodes and topics currently running.
#### Using `rqt_graph`
`rqt_graph` creates a dynamic graph of how different nodes in the system are communicating with each other.
In a new terminal:
```
$ rosrun rqt_graph rqt_graph
```
You will see something similar to:

![rqt_graph turtlesim](https://github.com/rachelruijiayang/curc_devfest/blob/master/resources/rqt_graph_turtle.PNG?raw=true)

As you can see, an arrow labeled with `/turtle1/cmd_vel` is going **from** the node `/teleop_turtle` **to** the node `/turtlesim`. This graph indicates that `/teleop_turtle` is **publishing** velocity commands that `/turtlesim` **subscribes** to.

#### Introducing `rostopic`
The rostopic tool allows you to get information about ROS **topics**.
You can use the help option to get the available sub-commands for `rostopic`
```
$ rostopic -h
```
Output:
```
rostopic bw     display bandwidth used by topic
rostopic echo   print messages to screen
rostopic hz     display publishing rate of topic    
rostopic list   print information about active topics
rostopic pub    publish data to topic
rostopic type   print topic type
```
Let's use some of these topic sub-commands to examine `turtlesim`.

#### Using `rostopic echo`
`rostopic echo` shows the data published on a topic. Usage:
```
$ rostopic echo [topic]
```
Let's look at the velocity commands published onto `/turtle1/cmd_vel` by the `turtle_teleop_key` node. Run in a separate terminal:
```
$ rostopic echo /turtle1/cmd_vel
```
You probably won't see anything happen because no data is being published on the topic yet. Recall that `turtle_teleop_key` converts input keystrokes into velocity commands for the turtle. Select the `turtle_teleop_key` terminal and use your arrow keys to steer the terminal around. When you press the up key, you should see the actual velocity commands being sent in the terminal where `rostopic echo /turtle1/cmd_vel` is running:
```
linear: 
  x: 2.0
  y: 0.0
  z: 0.0
angular: 
  x: 0.0
  y: 0.0
  z: 0.0
---
linear: 
  x: 2.0
  y: 0.0
  z: 0.0
angular: 
  x: 0.0
  y: 0.0
  z: 0.0
---
```
Press the left and right key, and observe how the commanded velocities change.

#### Using `rostopic list`
`rostopic list` returns a list of all topics currently subscribed to and published. `rostopic list -v` (verbose option) displays a list of topics that are being published to and subscribed to, and the type of the messages on each topic.
```
$ rostopic list -v
```
Output:
```
Published topics:
 * /turtle1/color_sensor [turtlesim/Color] 1 publisher
 * /turtle1/cmd_vel [geometry_msgs/Twist] 1 publisher
 * /rosout [rosgraph_msgs/Log] 2 publishers
 * /rosout_agg [rosgraph_msgs/Log] 1 publisher
 * /turtle1/pose [turtlesim/Pose] 1 publisher

Subscribed topics:
 * /turtle1/cmd_vel [geometry_msgs/Twist] 1 subscriber
 * /rosout [rosgraph_msgs/Log] 1 subscriber
```

### ROS Messages
Communication on topics happens by sending ROS **messages** between nodes. For the publisher (`turtle_teleop_key`) and subscriber (`turtlesim_node`) to communicate, the publisher and subscriber must send and receive the same **type** of message. This means that a topic **type** is defined by the message **type** published on it. The **type** of the message sent on a topic can be determined using `rostopic type`.

#### Using rostopic type
`rostopic type` returns the message type of any topic being published.
Usage:
```
rostopic type [topic]
```
Try:
```
$ rostopic type /turtle1/cmd_vel
```
You should get `geometry_msgs/Twist`.
We can look at the details of the message using `rosmsg`:
```
$ rosmsg show geometry_msgs/Twist
```
```
geometry_msgs/Vector3 linear
  float64 x
  float64 y
  float64 z
geometry_msgs/Vector3 angular
  float64 x
  float64 y
  float64 z
```
### Publishing to Topics Manually
`rostopic pub` allows you to manually create and publish messages onto a topic currently advertised.
Usage:
```
rostopic pub [topic] [msg_type] [args]
```
The following command will send a single message to `turtlesim` telling it to move with a forward (+x direction is forward) linear velocity of 1.0, and a counterclockwise rotation around the +z axis with angular velocity of 1.0. It will publish one such command per second, as given by `-r 1` (`-r` = rate).
```
$ rostopic pub /turtle1/cmd_vel geometry_msgs/Twist -r 1 -- '[1.0, 0.0, 0.0]' '[0.0, 0.0, 1.0]'
```
`[1.0, 0.0, 0.0]` represents the linear Vector3 component and `[0.0, 0.0, 1.0]` represents the angular Vector3 component of the Twist message.

You will see the turtle moving in a circle, as shown:
![enter image description here](http://wiki.ros.org/ROS/Tutorials/UnderstandingTopics?action=AttachFile&do=get&target=turtle%28rostopicpub%29.png)

You can now close your terminal windows from this exercise.

## Your Own Turtlesim Controller
We will now write a ROS node that continuously publishes to the `/turtle1/cmd_vel` topic, and therefore programmatically controls the simulated turtle.

To do so, we first need to create a ROS package for our code.

### Creating a ROS package
We will create a new package in our workspace `catkin_ws`. `catkin_create_pkg` is a command-line tool that automates some of the package-creation process.
```
$ cd ~/catkin_ws/src
$ catkin_create_pkg turtle_control_basic geometry_msgs roscpp
```
These commands will create the folder `turtle_control_basic`, which contains a `package.xml` (package manifest) and a `CMakeLists.txt` (!!! SEE LATER SECTION !!!), both of which have been partially filled out with the information you gave `catkin_create_pkg`. `catkin_create_pkg` also generates an `include` folder for header files and `src` folder for source code.

`catkin_create_pkg` requires that you give it a package name and optionally a list of dependencies on which that package depends: In our case, the package name is `turtle_control_basic` and the dependencies are:
- [`geometry_msgs`](http://wiki.ros.org/geometry_msgs): `geometry_msgs` provides messages for common geometric primitives such as points, vectors, and poses.
- [`roscpp`](http://wiki.ros.org/roscpp): This is a C++ implementation of ROS. `roscpp` is the most widely used ROS client library and is designed to be the high-performance library for ROS. ROS also has a Python API, but we do not use it in this tutorial. All ROS packages implemented in C++ depend on `roscpp`.

### Writing the Publisher Node
Now, let's write a node that makes the turtle spin in a circle clockwise.

Within `~/catkin_ws/src/turtle_control_basic/src`,  create a file with the name `turtle_control_basic_node.cpp` (full path `~/catkin_ws/src/turtle_control_basic/src/turtle_control_basic_node.cpp`).

The `turtle_control_basic_node.cpp` code will publish `geometry_msgs/Twist` messages to `/turtle1/cmd_vel`. Copy the following code into the `turtle_control_basic_node.cpp` file:
```cpp
#include "ros/ros.h"

#include "geometry_msgs/Twist.h"

int main(int argc, char **argv)
{	
	/**
	* You must call one of the versions of ros::init() before using any other
	* part of the ROS system.
	*/
	ros::init(argc, argv, "turtle_controller_basic");
	
	/**
	* NodeHandle is the main access point to communications with the ROS system.
	* The first NodeHandle constructed will fully initialize this node, and the last
	* NodeHandle destructed will close down the node.
	*/
	ros::NodeHandle n;
	
	/**
	* The advertise() function is how you tell ROS that you want to
	* publish on a given topic name. This invokes a call to the ROS
	* master node, which keeps a registry of who is publishing and who
	* is subscribing. After this advertise() call is made, the master
	* node will notify anyone who is trying to subscribe to this topic name,
	* and they will in turn negotiate a peer-to-peer connection with this
	* node.  advertise() returns a Publisher object which allows you to
	* publish messages on that topic through a call to publish().  Once
	* all copies of the returned Publisher object are destroyed, the topic
	* will be automatically unadvertised.
	*
	* The second parameter to advertise() is the size of the message queue
	* used for publishing messages.  If messages are published more quickly
	* than we can send them, the number here specifies how many messages to
	* buffer up before throwing some away.
	*/
	ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);
	
	ros::Rate loop_rate(10);

	while (ros::ok())
	{
		/**
		* This is a message object. You stuff it with data, and then publish it.
		*/
		geometry_msgs::Twist msg;
		// Set the fields of the message so that the turtle moves in a circle clockwise
		msg.linear.x = 1;
		msg.linear.y = 0;
		msg.linear.z = 0;
		msg.angular.x = 0;
		msg.angular.y = 0;
		msg.angular.z = -1;
		
		ROS_INFO("Published a geometry_msgs::Twist message with parameters\n\
			msg.linear.x = %f\n\
			msg.linear.y = %f\n\
			msg.linear.z = %f\n\
			msg.angular.x = %f\n\
			msg.angular.y = %f\n\
			msg.angular.z = %f\n\n\
		", msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.x,
		msg.angular.x, msg.angular.x);
		
		/**
	    * The publish() function is how you send messages. The parameter
	    * is the message object. The type of this object must agree with the type
	    * given as a template parameter to the advertise<>() call, as was done
	    * in the constructor above.
	    */
	    cmd_vel_pub.publish(msg);
	    ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
```

### The Code Explained 
Now, let's break the code down.
```cpp
#include "ros/ros.h"
```
`ros/ros.h` is a convenience include that includes all the headers necessary to use the most common public pieces of the ROS system.

```cpp
#include "geometry_msgs/Twist.h"
```
This includes the [`geometry_msgs/Twist`](https://docs.ros.org/api/geometry_msgs/html/msg/Twist.html) message, which resides in the [`geometry_msgs`](http://wiki.ros.org/geometry_msgs) package. This is a header generated automatically from the `Twist.msg` file in that package. For more information on message definitions, see the [`msg`](http://wiki.ros.org/msg) page.

```cpp
ros::init(argc, argv, "turtle_control_basic_node");
```
Initialize ROS. This allows ROS to do name remapping through the command line -- not important for now. This is also where we specify the name of our node. Node names must be unique in a running system.
The name used here must be a base name, i.e. it cannot have a / in it.

```cpp
ros::NodeHandle n;
```
Create a handle to this process' node. The first NodeHandle created will actually do the initialization of the node, and the last one destructed will cleanup any resources the node was using.

```cpp
ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);
```
Tell the master that this node is going to be publishing a message of type geometry_msgs/Twist on the topic `/turtle1/cmd_vel`. This lets the master tell any nodes listening on `/turtle1/cmd_vel` that this node is going to publish data on that topic. The second argument is the size of our publishing queue. In this case if we are publishing too quickly it will buffer up a maximum of 1000 messages before beginning to throw away old ones.
`NodeHandle::advertise()` returns a `ros::Publisher object`, which serves two purposes: 1) it contains a `publish()` method that lets you publish messages onto the topic it was created with, and 2) when it goes out of scope, it will automatically unadvertise.

```cpp
ros::Rate loop_rate(10);
```
A `ros::Rate` object allows you to specify a frequency that you would like to loop at. It will keep track of how long it has been since the last call to `Rate::sleep()`, and sleep for the correct amount of time. In this case, the while loop execute 10 times per second.

```cpp
while (ros::ok())
{
```
By default roscpp will install a SIGINT handler which provides Ctrl-C handling which will cause `ros::ok()` to return false if that happens.

`ros::ok()` will return false if:
- a SIGINT is received (Ctrl-C)
- we have been kicked off the network by another node with the same name
- ros::shutdown() has been called by another part of the application.
- all ros::NodeHandles have been destroyed
Once `ros::ok()` returns false, all ROS calls will fail.

```cpp
		geometry_msgs::Twist msg;
		// Set the fields of the message so that the turtle moves in a circle clockwise
		msg.linear.x = 1;
		msg.linear.y = 0;
		msg.linear.z = 0;
		msg.angular.x = 0;
		msg.angular.y = 0;
		msg.angular.z = -1;
```
We constuct the message to be broadcasted. geometry_msgs::Twist is a C++ struct type that was included using `#include "geometry_msgs/Twist.h"`.  This header file was generated from the msg file for `geometry_msgs::Twist`. The variable `msg` is a specific instance of a `geometry_msgs::Twist` struct, and its struct fields can be set individually.

```cpp
		cmd_vel_pub.publish(msg);
```
Now we actually broadcast the message to anyone who's connected to `/turtle1/cmd_vel`, the topic that `cmd_vel_pub` publishes on.

```cpp
		ROS_INFO("Published a geometry_msgs::Twist message with parameters\n\
			msg.linear.x = %f\n\
			msg.linear.y = %f\n\
			msg.linear.z = %f\n\
			msg.angular.x = %f\n\
			msg.angular.y = %f\n\
			msg.angular.z = %f\n\n\
		", msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.x,
		msg.angular.x, msg.angular.x);
```
`ROS_INFO` is ROS's replacement for `printf`/`cout`.

```cpp
		ros::spinOnce();
```
Calling `ros::spinOnce()` here is not necessary for this simple program, because we are not receiving any callbacks. However, if you were to add a subscription into this application, and did not have `ros::spinOnce()` here, your callbacks would never get called. So, add it for good measure.

```cpp
		loop_rate.sleep();
```
Now we use the `ros::Rate` object to sleep for the time remaining to let us hit our 10Hz publish rate.

Here's the condensed version of what's going on:
- Initialize the ROS system
- Advertise that we are going to be publishing `geometry_msgs/Twist` messages on the `/turtle1/cmd_vel` topic to the master
- Loop while publishing messages to `/turtle1/cmd_vel` 10 times a second

### Building your node using catkin_make
How do we compile our source code, which depends on code from the ROS packages `roscpp` and `geometry_msgs`, into an executable?

[Catkin](http://wiki.ros.org/catkin/conceptual_overview) is the official build system of ROS. Catkin extends [CMake](https://en.wikipedia.org/wiki/CMake) to manage dependencies between packages. The file `CMakeLists.txt` is the input to the CMake build system for building software packages describes how to build the code for a ROS package and where to install it to. We will now modify the CMakeLists.txt file within our `turtle_control_basic` package.

(Note: The name catkin comes from the tail-shaped flower cluster found on willow trees -- a reference to Willow Garage where catkin was created!)

If you open `turtle_control_basic`'s CMakeLists.txt file in a text editor, you will see a 200-line file containing many options commented out using `#`. Because the auto-generated CMakeLists.txt file is crowded with comments and unnecessary options, please delete all of the text in the file. Then, then paste the following text into your `CMakeLists.txt`:
```
# What version of CMake is needed?
cmake_minimum_required(VERSION 2.8.3)

# Name of this package
project(turtle_control_basic)

# Find the catkin build system, and any other packages on which we depend
find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs)

## Declare our catkin package
catkin_package()

# Specify locations of header files
include_directories(include ${catkin_INCLUDE_DIRS})

# For each executable provided by this package:
# add_executable: Declare the executable by specifying its source files.
# target_link_libraries: Specify libraries against which to link.
add_executable(turtle_control_basic_node src/turtle_control_basic_node.cpp)
target_link_libraries(turtle_control_basic_node ${catkin_LIBRARIES})
```
This will create an executable `turtle_control_basic_node`, which by default will go into `<workspace name>/devel/lib/<package name>`. In our case, the executable will go into `~/catkin_ws/devel/lib/turtle_control_basic`.

The final step is to go to the root of your workspace, run `catkin_make` (and watch your programs build and link in pretty colors!), and then refresh your bash environment variables:
```
$ cd ~/catkin_ws
$ catkin_make
$ source ~/.bashrc
```
You can run your executable using:
```
$ rosrun turtle_control_basic turtle_control_basic_node
```
Start up a `turtlesim` node in another terminal window using:
```
$ rosrun turtlesim turtlesim_node
``` 
And watch that turtle spin!

### Try out the following:
 - How would you change `turtle_control_basic_node.cpp` to make the turtle move faster?
 - How would you change `turtle_control_basic_node.cpp` to make the turtle periodically switch its direction?
- What if you run `turtle_control_basic_node` and control the turtle with arrow keys through `turtle_telop_key` at the same time? Try it out while, and with both nodes running, run `rosrun rqt_graph rqt_graph` to see the computation graph.

Congratulations! You just built a program to make a robot move! However, at the moment, the robot is just moving on its own, not in response to any sensor input.

In the next chapter, you'll learn the basics of the computer vision library OpenCV. Computer vision will allow the robot to process sensor (camera) input in a useful way.

<a href="#top" class="top" id="level2"></a>
# Level 2: Basic OpenCV
To skip to the start of Level 2: `git checkout level2`

In this level, you will implement computer vision using OpenCV so that your robot can perceive and interpret the real world!

## Setting up the webcam
We need to allow your Virtual Machine to access the webcam. While your VM is started, on Devices, Webcams, and click on the camera that pops up:
![enter image description here](https://github.com/rachelruijiayang/curc_devfest/blob/master/resources/webcam_ensure.png?raw=true)
Next, let's verify that that the webcam is connected to your VM. Go to the Ubuntu Start menu and search for “Cheese Webcam Booth”. Click on the app. Now, you should be able to take a selfie.

# Level 3: Using CV Output to Actuate the Turtle
To skip to the start of Level 3: `git checkout level3`
# Level 4: Face Detection using OpenCV
To skip to the start of Level 4: `git checkout level4`
# Level 5: What’s next?
To skip to the start of Level 5: `git checkout level5`
