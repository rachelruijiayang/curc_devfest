
<a id="top"></a>
# CURC DevFest Robotics Track
*How to build robots using Robot Opeating System, OpenCV, and other ROS packages.*

Written and developed by [Rachel Yang](https://github.com/rachelruijiayang), [Ignacio Ramirez-Piriz](https://github.com/ir2331), and [Columbia University Robotics Club](http://www.columbia.edu/cu/roboticsclub/). 

Portions of this tutorial are adapted from [NVIDIA's Robotics Teaching Kit with 'Jet'](https://developer.nvidia.com/teaching-kits), [*ROS Robotics Projects* by Lentin Joseph](https://www.amazon.com/ROS-Robotics-Projects-Lentin-Joseph/dp/1783554711) (available to Columbia students through [CLIO](https://clio.columbia.edu/quicksearch?q=ros+robotics+projects)), and [*Effective Robotics Programming with ROS - Third Edition* by Anil Mahtani and Luis Sanchez](https://www.amazon.com/Effective-Robotics-Programming-ROS-Third/dp/1786463652) (also available through [CLIO](https://clio.columbia.edu/quicksearch?q=effective+robotics+programming+with+ros+third+edition&commit=Search)).

## About this Tutorial

This tutorial differs from other DevFest tracks in that it does not work towards one specific final product. Instead, Levels 0 - 2 provide learners with an environment setup and practice with ROS and computer vision basics.

Levels 1 and 2 have their own branch in the [GitHub repository](https://github.com/rachelruijiayang/catkin_ws) for our ROS workspace (`~/catkin_ws`). If you don't want  to do Level 1, go to `~/catkin_ws` and `git checkout level2`. If you don't want to do Level 2, `git checkout adventure`.

After Level 2, you will have the skills to work on projects of your choice from Lentin Joseph's *ROS Robotics Projects*. We could not reproduce the contents of that book in this tutorial for copyright reasons, but the book is available to Columbia students for free through [CLIO](https://clio.columbia.edu/quicksearch?q=ros+robotics+projects). We've starred our favorite projects and provided tips and adjustments for them [here](#rrp_table_of_contents).

## Why learn ROS?
ROS is an open-source framework for building robots. 

A robot is a system in which sensors (such as cameras) connect to processors, which connect to actuators to make the robot move (or make some other decision). ROS provides a publish-subscribe messaging infrastructure that makes it easier for different parts of a robot system to talk to each other. In Levels 1 and 2 of this tutorial, you will write your own ROS publisher and subscriber in C++.

ROS also provides an extensive set of tools for configuring, starting, debugging, simulating, testing, and stopping robot systems. In this tutorial, you will familiarize yourself with the tools  rqt_graph, rostopic, rospub, rosnode, and more.

Finally, ROS provides a broad collection of open-source libraries that implement useful robot functionality, with a focus on mobility, manipulation, and perception, and a large active developer community. [[Source]](https://answers.ros.org/question/12230/what-is-ros-exactly-middleware-framework-operating-system/) Level 2 of this tutorial walks you through the basics of one ROS-integrated library, OpenCV. After Level 2, you will have the skills to play with open source ROS packages and integrations for chatbots, deep learning, and more.

[Here are some past robots that have been built using ROS.](https://www.youtube.com/watch?v=Z70_3wMFO24)

## Prerequisites

 - A computer with a webcam (required for the computer vision sections)
 - At least 25 GB of free storage space on your computer (to install the VM)
 - Prior programming experience, especially in C++ and Python
 - Familiarity with using the Linux terminal

# Table of Contents
- [Level 0: Environment Setup](#level0)
- [Level 1: Intro to ROS](#level1)
- [Level 2: Basic OpenCV](#level2)
- [Choose Your Own Adventure: *ROS Robotics Projects*](#rrp_table_of_contents)

<a href="#top" class="top" id="level0"></a>
# Level 0: Environment Setup
To develop in the [ROS Kinetic](http://wiki.ros.org/kinetic), you will need to be running Ubuntu 16.04 as your OS. We will be using [VirtualBox](https://www.virtualbox.org/manual/ch01.html) to run a custom Ubuntu 16.04 VM image that already has ROS and the other dependencies for this tutorial installed on it.

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
**Username: `robot`
Password: `robot`**

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
In this level, you will learn the structure and fundamental building blocks of ROS. Then, using those concepts, you'll write (in C++) your own ROS node to programmatically control a simulated turtle. 

ROS has three levels of concepts: the Filesystem level, the Computation Graph level, and the Community level. These levels and the most important components of them are summarized below. See http://wiki.ros.org/ROS/Concepts for the full list.

## The ROS Filesystem Level
The filesystem level concepts mainly cover ROS resources that you encounter on disk, such as:

<a href="#top" class="top" id="quiz_workspace"></a>
- **Workspaces**: A workspace is a folder where you modify, build, and install ROS packages (also called catkin packages). 
	- `~/catkin_ws` is the workspace for many of the ROS packages we will write and build in this tutorial.
<a href="#top" class="top" id="quiz_package"></a>
- **Packages**: Packages are the main unit for organizing software in ROS. Generally, each package implements some particular functionality that can be integrated into a larger system. A package may contain ROS runtime processes (nodes), a ROS-dependent library, datasets, configuration files, or anything else that is usefully organized together. The most granular thing you can build and release is a package.
	- If you `cd` into `~/catkin_ws/src`, you will see a directory called `usb_cam`. `usb_cam` is a ROS package (documentation: http://wiki.ros.org/usb_cam). The functionality of `usb_cam` is to interface with standard USB cameras (including webcams) and convert a raw image stream from the camera into a stream of ROS image messages.
- **Package Manifests**: In the root of each ROS package, there is a file called package.xml, which is the package manifest. Manifests provide metadata about a package, including its name, version, description, license information, dependencies, and other meta information like exported packages.
	- Check out the manifest for `usb_cam` at `~/catkin_ws/src/usb_cam/package.xml`
- **Message (msg) types**: Message descriptions, stored in {package_name}/msg/{my_message_type}.msg, define the data structures for messages sent in ROS.
	- `usb_cam` has a single node (exectuable) that converts images from the USB camera into ROS messages of type `sensor_msgs/Image`. [Documentation for `sensor_msgs/Image`](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html)

## The ROS Computation Graph
A robot system can be thought of as a peer-to-peer network of sensors, processors, and actuators. Each of these components will run a ROS process. The ROS Computation Graph is the peer-to-peer network of ROS processes that make up one robot system and are processing data together. The basic Computation Graph concepts of ROS are nodes, Master, Parameter Server, messages, and topics.

![ROS Computation Graph](http://ros.org/images/wiki/ROS_basic_concepts.png)
<a href="#top" class="top" id="quiz_node"></a>
- **Nodes**: Nodes are processes that perform computation. ROS is designed to be modular at a fine-grained scale; a robot control system usually comprises many nodes. For example, one node controls a laser range-finder, one node controls the wheel motors, one node performs localization, one node performs path planning, one Node provides a graphical view of the system, and so on. A ROS node is written with the use of a ROS client library, such as roscpp or rospy.
<a href="#top" class="top" id="quiz_master"></a>
- **Master**: The ROS Master provides name registration and lookup to the rest of the Computation Graph. Without the Master, nodes would not be able to find each other, exchange messages, or invoke services.
- **Messages**: Nodes communicate with each other by passing messages. A message is simply a data structure, comprising typed fields. Standard primitive types (integer, floating point, boolean, etc.) are supported, as are arrays of primitive types. Messages can include arbitrarily nested structures and arrays (much like C structs).
<a href="#top" class="top" id="quiz_topic"></a>
- **Topics**: Messages are routed via a transport system with publish / subscribe semantics. A node sends out a message by **publishing** it to a given topic. The topic is a name that is used to identify the content of the message. A node that is interested in a certain kind of data will **subscribe** to the appropriate topic. There may be multiple concurrent **publishers** and **subscribers** for a single topic, and a single node may publish and/or subscribe to multiple topics. In general, publishers and subscribers are not aware of each others' existence. The idea is to *decouple the production of information from its consumption*. Logically, one can think of a topic as a strongly typed message bus. Each bus has a name, and anyone can connect to the bus to send or receive messages as long as they are the right type.
<a href="#top" class="top" id="quiz_param"></a>
- **ROS Parameters/Parameter Server**: ROS parameters are another mechanism ROS provides to get information to nodes. The idea is that a centralized Parameter Server keeps track of a collection of values—things like integers, floating point numbers, strings, or other data—each identified by a short string name. Because parameters must be actively queried by the nodes that are interested in their values, they are most suitable for configuration settings that will not change (much) over time." [Source](https://cse.sc.edu/~jokane/agitr/agitr-small-param.pdf). 

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
You will see the `turtlesim` window:

![enter image description here](http://wiki.ros.org/ROS/Tutorials/UnderstandingNodes?action=AttachFile&do=get&target=turtlesim.png)

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
Press the left and right key, and observe that the angular velocity changes.

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

<a href="#top" class="top" id="quiz_message"></a>
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

Within `~/catkin_ws/src/turtle_control_basic/src`,  create a file with the name `turtle_control_basic_node.cpp` (full path `~/catkin_ws/src/turtle_control_basic/src/turtle_control_basic_node.cpp`). The text editor Gedit comes with Ubuntu, but you can choose to download your own text editor. You can open a file for editing in Gedit by running `gedit [filename]`.

<a href="#top" class="top" id="quiz_pub_example"></a>
The `turtle_control_basic_node.cpp` code will **publish** `geometry_msgs/Twist` messages to `/turtle1/cmd_vel`. Copy the following code into the `turtle_control_basic_node.cpp` file:
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
`ros::spinOnce()` will call all the callbacks waiting to be called at that point in time. `ros::spinOnce()` here is not necessary for this simple program, because we are not receiving any callbacks. However, if you were to add a subscription into this application, and did not have `ros::spinOnce()` here, your callbacks would never get called. So, add it for good measure.

```cpp
		loop_rate.sleep();
```
Now we use the `ros::Rate` object to sleep for the time remaining to let us hit our 10Hz publish rate.

Here's the condensed version of what's going on:
- Initialize the ROS system
- Advertise that we are going to be publishing `geometry_msgs/Twist` messages on the `/turtle1/cmd_vel` topic to the master
- Loop while publishing messages to `/turtle1/cmd_vel` 10 times a second


<a href="#top" class="top" id="quiz_compile"></a>
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

<a href="#top" class="top" id="quiz_sub"></a>
### Subscribers
The node you built publishes to `/turtle1/cmd_vel`, but does not subscribe to any topics. In contrast, `turtlesim_node` subscribes to  `/turtle1/cmd_vel`.  Upon receiving each `geometry_msgs::Twist` message on `/turtle1/cmd_vel`, `turtlesim_node` calls a **callback function** to moves itself with the commanded velocity.

#### Callback Functions
[Source](https://cse.sc.edu/~jokane/agitr/agitr-letter-pubsub.pdf)
One important difference between publishing and subscribing is that a subscriber node doesn’t know when messages will arrive. To deal with this fact, we must place any code that responds to incoming messages inside a callback function, which ROS calls once for each arriving message. A subscriber callback function looks like this:
```cpp
void function_name(const package_name::type_name &msg) {
/* take some action based on the contents of the message */
}
```
The package_name and type_name are the same as for publishing: They refer to the message class for the topic to which we plan to subscribe. The body of the callback function then has access to all of the fields in the received message, and can store, use, or discard
that data as it sees fit.

In Level 2, you will write a callback function that does a computer vision transformation on each image that arrives on a topic that your node subscribes to. 

### Try out the following:
 - How would you change `turtle_control_basic_node.cpp` to make the turtle move faster? How would you change the code to make the turtle spin in a circle with a larger radius?
 - How would you change `turtle_control_basic_node.cpp` to make the turtle periodically switch its direction?
- What if you run `turtle_control_basic_node` and control the turtle with arrow keys through `turtle_telop_key` at the same time? Try it out while, and with both nodes running, run `rosrun rqt_graph rqt_graph` to see the computation graph.
- Check out the implementation of `turtlesim_node` here: https://github.com/ros/ros_tutorials/tree/kinetic-devel/turtlesim/src. The turtle's publishers and subscribers are defined in `turtle.cpp`. (In that file, Ctrl+F for "advertise" and "subscribe".) How are turtlesim_node's publishers and subscribers defined?

Congratulations! You just built a program to make a robot move! However, at the moment, the robot is just moving on its own, not in response to any sensor input.

In the next chapter, you'll learn the basics of the computer vision library OpenCV. Computer vision will allow the robot to process sensor (camera) input in a useful way.

<a href="#top" class="top" id="level2"></a>
# Level 2: Basic OpenCV
Source: [NVIDIA's Robotics Teaching Kit with 'Jet'](https://developer.nvidia.com/teaching-kits)

To skip to the start of Level 2: `git checkout level2`

In this lab, you will explore a variety of techniques in computer vision. These methods can be used in robot systems to track objects, locate items, or detect obstacles.
#### Learning Outcomes
- Read images from ROS
- Explore image representations
- Blur images
- Detect edges
- Hough Line Transform
- Image moments

## Setting Up Your Camera
### Webcam Access in VirtualBox
We need to allow your Virtual Machine to access the webcam. While your VM is started, on Devices, Webcams, and click on the camera that pops up:

![enter image description here](https://github.com/rachelruijiayang/curc_devfest/blob/master/resources/webcam_ensure.png?raw=true)
Next, let's verify that that the webcam is connected to your VM. Go to the Ubuntu Start menu and search for “Cheese Webcam Booth”. Click on the app.  You should be able to take a selfie!

### Interfacing the Webcam with ROS
Let's test the webcam using the usb_cam package. The following command is used to launch the usb_cam nodes to display images from a webcam and publish ROS image topics at the same time:
```
roslaunch usb_cam usb_cam-test.launch
```
If everything works fine, you will be able to view the camera input, and logs will be printed to the terminal. Press Ctrl+C in the terminal window to close the image view.

#### Note: 
If you get random-colored pixels above a green background, it is likely because your webcam outputs a different format than the default that this tutorial's `usb_cam` package expects:
![enter image description here](https://github.com/rachelruijiayang/curc_devfest/blob/master/resources/yuyv_error.PNG?raw=true)
Change the `usb_cam` settings by running the following commands, **only if you are getting the erroneous display above**:
1. Enter the directory of the `usb_cam` ROS package, and then enter the launch folder:
	`roscd usb_cam && cd launch`
2. Edit the file `usb_cam-settings.launch`:
	`gedit usb_cam-settings.launch`
3. Find the line that says `<param name="pixel_format" value="mjpeg" />`. Change `mjpeg` to `yuyv`. Save the file.
	- Note: You can check the pixel format that your webcam outputs by running in a terminal `v4l2-ctl --list-formats-ext | grep Pixel`. If this command outputs "MJPG", `value` should be set to `mjpeg`.
4.	Relaunch `usb_cam-test.launch` by running `roslaunch usb_cam usb_cam-test.launch`

You have finished the set up for this level; continue to learn about basic computer vision!

## Basic CV
This section will use `basic_opencv/src/basic_cv.cpp`. 

### Scaffolding Code Explained
```cpp
cv::Mat src, gray, edges, dst, cdst;
vector<Vec4i> lines;

image_transport::Publisher user_image_pub;
image_transport::Subscriber raw_image_sub;
```
- [`cv::Mat`](https://docs.opencv.org/2.4/doc/tutorials/core/mat_the_basic_image_container/mat_the_basic_image_container.html) exports an n-dimensional dense numerical single-channel or multi-channel array.  It can be used to store grayscale images, color images, vectors, matrices, and more. Here, we declare five cv::Mat objects.
- [`Vec4i`](https://docs.opencv.org/trunk/dc/d84/group__core__basic.html#ga94ce799099ae6cdd66685e3fd0cad7d7) is a structure for representing a vector with 4 dimensions, with each value an int.
- `user_image_pub` publishes the images that are transformed by the code in this ROS node
- `raw_image_sub` is a subscriber for listening to raw images from `usb_cam`

You can read more about `image_transport` [here](http://wiki.ros.org/image_transport).

Our ROS loop is in the main function. The `usb_cam` ROS node is responsible for capturing video frames from the webcam and publishing them to a ROS topic called `/usb_cam/image_raw`. Any node that needs to access the video frames can subscribe to this topic. The ImageTransport framework is used to publish and subscribe to image topics. The code below shows how you can subscribe to the raw images on `/usb_cam/image_raw` and publish your own images on `/user/image1`. Often it is useful when building computer vision applications to visualize how your algorithm is performing. For example, if you are detecting faces, then you can draw circles around each face that is detected then publish this modified image to a new image topic.
```cpp
int main(int argc, char **argv)
{
  ros::init(argc, argv, "basic_cv");
  ros::NodeHandle nh;

  image_transport::ImageTransport it(nh);

  //advertise the topic with our processed image
  user_image_pub = it.advertise("/user/image1", 1);

  //subscribe to the raw usb camera image
  raw_image_sub = it.subscribe("/usb_cam/image_raw", 1, imageCallback);
  
  ros::spin();
}
```
Note: [`ros::spin()`](http://wiki.ros.org/roscpp/Overview/Callbacks%20and%20Spinning) does not return until the node is shutdown. All user callbacks can be called from within the `ros::spin()` call.

<a href="#top" class="top" id="quiz_sub_example"></a>
Like other types of topics, you subscribe to image topics using a callback. The `imageCallback` function referenced above can be implemented like the code below. The code in this callback function is run every time a the subscriber `raw_image_sub` reads a new image on the `/usb_cam/image_raw` topic.
```cpp
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {

    src = cv_bridge::toCvShare(msg, "bgr8")->image;

    /*
     * INSERT CODE HERE
    */

    sensor_msgs::ImagePtr msg;
    msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", src).toImageMsg();

    user_image_pub.publish(msg);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}
```
In this case, the `cv_bridge::toCvShare` function is used to convert the image topic message to a matrix that OpenCV can use. These OpenCV matrices are grids where each cell has blue, green, and red value. The code above does not modify the image, but merely republishes the same image using the `user/image1` topic. `cv_bridge` is a ROS package that [converts between ROS image messages and OpenCV images](http://wiki.ros.org/cv_bridge).

<a href="#top" class="top" id="compile_basic_cv"></a>
You can compile the basic_cv node (and all other new source code in your `catkin_ws`) using:
```
$ cd ~/catkin_ws
$ catkin_make
$ source ~/.bashrc
```
Run this node with the launch script:
```
$ roslaunch basic_opencv basic_cv.launch
```
Then run image_viewer:
```
$ rosrun image_view image_view image:=/user/image1
```
The image that `image_view` reads from `/user/image1` should be the same as the image from the raw camera, because we have not yet used any OpenCV filters within our source code file. In the next section, you will call OpenCV functions to manipulate the input image and publish modified images on `/user/image1`.

<a href="#top" class="top" id="quiz_launchfile"></a>
### Launch Files
Earlier, you used `roslaunch` to launch nodes specified in the **launch file** `usb_cam-test.launch`, part of the `usb_cam` package.  Just now, you used `roslaunch` to launch the **launch file** `basic_cv.launch`, part of the `basic_cv` package. `roslaunch` is a tool for easily launching multiple ROS nodes at once. Many ROS packages come with "launch files", which you can run with:
```
$ roslaunch [package_name] [file.launch]
```
`roslaunch` uses XML files that describe the nodes that should be run, parameters that should be set, and other attributes of launching a collection of ROS nodes. 

Let's take a look at `basic_cv.launch` (in `~/catkin_ws/src/basic_opencv/launch/basic_cv.launch) to see what nodes it launches:
```xml
<launch>
  <include file="$(find usb_cam)/launch/usb_cam-settings.launch"/>
  <node name="basic_cv" pkg="basic_opencv" type="basic_cv"/>
</launch>
```
#### Launch File  Explained
([Source: *A Gentle Introduction to ROS* - Jason M. O'Kane.](https://cse.sc.edu/~jokane/agitr/agitr-small-launch.pdf))
**Inserting the root element**: Launch files are XML documents, and every XML document must have exactly one **root element**. For ROS launch files, the root element is defined by a pair of `launch` tags:
```xml
<launch>
...
</launch>
```
All the other elements of each launch file should be enclosed between these tags.

**Launching nodes**: The heart of any launch file is a collection of node elements, each of which names a single node to launch. A node element looks like this:
```xml
<node
	name="node-name"
	pkg="package-name"
	type="executable-name"
/>
```
In our case:
```xml
<node name="basic_cv" pkg="basic_opencv" type="basic_cv"/>
```

**Including other files**: To include the contents of another launch file, including all of its nodes and parameters, use an include element:
```xml
<include file="path-to-launch-file" />
```
The file attribute expects the full path to the file we want to include. Because it can be both cumbersome and brittle to enter this information directly, most include elements
use a find substitution to search for a package, instead of explicitly naming a directory:
```xml
<include file="$(find package-name)/launch-file-name" />
```
In our case:
```xml
<include file=$(find usb_cam)/launch/usb_cam-settings.launch"/>
```
### Basic Transformations

#### Grayscale
For some applications, it is easier to work with images that are grayscale rather than full-color. In OpenCV, the `cvtColor` function is used for this conversion. Add the lines below to the "INSERT CODE HERE" section of basic_cv.cpp. **Then replace the variable `src` with `cdst` in the `cv_bridge::CvImage` function call**.
```cpp
cvtColor(src, gray, CV_BGR2GRAY);
cvtColor(gray, cdst, CV_GRAY2BGR);
```
This code snippet makes `dst` into a grayscale matrix representation of the original `src` matrix.  `dst` has a single 8-bit value for each pixel, while `src` has three 8-bit values for each pixel (blue, green, red).  The ROS ImageMessage type requires images to be represented as a matrix of three-tuples, so it is necessary to make a new matrix `cdst` that is a black and white representation using three-tuples (three 8-bit values).

Compile and run the `basic_cv` node [(refresher on compilation)](#compile_basic_cv) and view the `user/image1` topic with `image_view` to see the grayscale image.

#### Blurring
Smoothing images is used in computer vision to reduce noise and make it easier to extract features.  Blurring an image assigns each pixel to the weighted sum of the nearby pixels.  There are a variety of averaging techniques that can be used.  We will look at two of them - the normalized box blur and the Gaussian Blur.

In the normalized box blur, a simple numerical average of nearby pixels is used.  The kernel size determines how many pixels are averaged for each value.  The `blur` function is used to perform a normalized box blur.  You can test it out using the following code segment.  Replace the `cvtColor` functions from the previous section with the following code:
```
blur(src, cdst, Size(3,3));
```
Compile and run the code to view the results.  The kernel is specified in the third argument to `blur`.  This value must be an odd number; the larger the value, the more smooth the final image will be.

The Gaussian Blur averages nearby pixel values according to a Gaussian distribution, so the closest pixels are weighted more heavily than the distance pixels.  Replace the normalized box blur with the Gaussian Blur function below to see the difference.
```
GaussianBlur(src, cdst, Size(3,3), 0, 0);
```
The `GaussianBlur` function's third argument is the kernel size, just like the normalized box blur.  The fourth and fifth parameters are the standard deviations for the x and y directions; leaving these values zero means that they will be computed from the kernel size.  Compile and run the `basic_cv` node with different kernel sizes (values must be odd but not necessarily the same).

[Compile and run the `basic_cv` node (along with `image_view`) again to see the results.](#compile_basic_cv) 

### Edge Detection
Edges in an image are the places where one colored region touches a different colored region.  Edge detection is used to understand the structure of an image without selecting specific color values.  The Canny Edge Detection algorithm is one of the most popular ways of extracting edges from an image.  In OpenCV, this algorithm is implemented in the `Canny` function.  Edge detection requires a black and white image.  Edge detection can be susceptible to noise, so we will blur the image prior to applying the edge detection algorithm.

Remove the blurring function calls from the previous section.  Then add a function call that creates a grayscale matrix called `gray` from the `src` matrix.  Next, apply a normalized box blur to the `gray` image and save the resulting blurred image to the `dst` matrix.  Now you can add the following code that computes the edges of the image and saves them to the `edges` matrix.  Finally, convert the `edges` matrix to a BGR matrix called `cdst`.  Compile and run the `basic_cv` node to see white lines representing edges in the original image.

```
Canny(dst, edges, 50, 200, 3);
```

The Canny algorithm has three important parameters (third, fourth, and fifth arguments, respectively): lower threshold, upper threshold, and the aperture size.  The threshold parameters define which pixels should be included as edge pixels.  The aperture size is used in the filtering of the image.  Smaller threshold values cause more pixels to be classified as edges. Try changing the lower threshold to 10 and upper threshold to 100 and rerunning `basic_cv`.

[Compile and run the `basic_cv` node (along with `image_view`) again to see the results.](#compile_basic_cv) 

### Hough Transforms
Edges can be thought of as image features that are more abstract than raw pixel values because they are independent of the exact coloring.  Even more abstract features like shapes can also be extracted from images.  The Hough Line Transform is a popular technique for detecting the lines in an image.  A similar procedure can be used for detecting circles and ellipses.  In this section, we will use OpenCV's `HoughLinesP` function to visualize the lines in an image.

Add the following code after the `Canny` function call.  Compile and run the `basic_cv` node.  Hold a straight edge in front of the camera; red lines indicate the lines that were detected by the algorithm.
```
HoughLinesP(edges, lines, 1, CV_PI/180, 80, 30, 10 );
for( size_t i = 0; i < lines.size(); i++ )
{
    line(cdst, Point(lines[i][0], lines[i][1]),
        Point(lines[i][2], lines[i][3]), Scalar(0,0,255), 2, 8 );
}
```
The `HoughLinesP` function saves each line to the lines vector.  Each line entry is four-tuple, which has two points (a start and end of the line).  Each line entry is composed of (x1, y1, x2, y2).  The final three arguments are the important values for fine-tuning the line detection.  The fifth value is the minimum threshold (the lower the value, the more lines will be found).  The sixth value is the minimum line length, and the seventh value is the minimum gap between lines.  Change the minimum line length to 5 to detect many more lines.

[Compile and run the `basic_cv` node (along with `image_view`) again to see the results.](#compile_basic_cv) 

## Image Tracking
In this section you will use the `vision_tracking` node.  You can run this node using roslaunch:
```
$ roslaunch basic_opencv vision_tracking.launch
```
The `vision_tracking` node will identify and track a colored object.  Choose an object that is brightly colored and easy to move (a small ball is a good choice).

### Topic 1: HSV Segmentation

We have already looked at the BGR and grayscale representations of images, and now we will examine another way of representing images - HSV (Hue, Saturation, and Value).  Like BGR, HSV matrices have three numbers.  The first number, the Hue, is an 8-bit integer that corresponds to the color of the pixel.  The second number, the saturation, is an 8-bit integer that corresponds to the whiteness of the pixel.  The third number, the value, corresponds to the lightness of the pixel.  Computer vision applications that must deal with colors generally prefer the HSV representation because the color of each pixel is encoded in a single number (Hue) rather than three in the BGR representation.

Our first step in building a tracking program is to find the pixels that are in the object. To do this, we will perform color-based segmentation.  Segmentation is the process of extracting certain components from the image.  The code below creates the `HSV` matrix using the HSV representation of the original image.   Then two scalar values are instantiated.  Finally, the `inRange` function is called.  The `inRange` function call sets the `mask` matrix's pixels to 255 when the corresponding `HSV` matrix pixel is within the range of the two thresholds and to 0 when it is outside the range.  Add the code below to the `imageCallback` function.
```cpp
//Convert the image to HSV
cv::cvtColor(src, hsv, CV_BGR2HSV);

//Define the range of blue pixels
cv::Scalar lower_thresh(hue_lower, sat_lower, value_lower);
cv::Scalar upper_thresh(hue_upper, sat_upper, value_upper);

//Create a mask with only blue pixels
cv::inRange(hsv, lower_thresh, upper_thresh, mask);

//Conert the image back to BGR
cv::cvtColor(mask, dst, CV_GRAY2BGR);
```
We left the hue, saturation, and value parameters for the upper and lower thresholds as variables, so we can dynamically change them while the node is running. `hue_lower`,  `hue_upper`,  `sat_lower`,  `sat_upper`, `value_lower`, and `value_upper` are  actually all set as ROS parameters. Recall our introduction [here](#quiz_param) about ROS parameters. In the case of the `vision_tracking` program, the Parameter Server keeps track of a collection of integers, each identified by a short string name, that function as "settings" for our vision tracker that can be changed at runtime. These parameters and their default values are specified in `cfg/Hue.cfg` of our `basic_opencv` package:
```python
#!/usr/bin/env python
PACKAGE = "basic_opencv"

from dynamic_reconfigure.parameter_generator_catkin import *

gen = ParameterGenerator()

gen.add("hue_upper", int_t, 0, "hue_upper", 120, 0, 180)
gen.add("hue_lower", int_t, 0, "hue_lower", 80, 0, 180)
gen.add("sat_upper", int_t, 0, "sat_upper", 255, 0, 255)
gen.add("sat_lower", int_t, 0, "sat_lower", 0, 0, 255)
gen.add("value_upper", int_t, 0, "value_upper", 255, 0, 255)
gen.add("value_lower", int_t, 0, "value_lower", 0, 0, 255)

exit(gen.generate(PACKAGE, "basic_opencv", "Hue"))
```
Then, after the node has been initialized, values of the parameters are read into global variables.
```cpp
int main(int argc, char **argv)
{
  ros::init(argc, argv, "vision_tracking");
  ros::NodeHandle nh;

  ros::NodeHandle nh_("~");

  // The method calls below have the form nh_.param<type>("specified_parameter", variable_to_store_parameter_value_in, default_value)
  // The NodeHandle method below gets the parameter values for hue_lower, hue_upper, etc. and stores them
  // into the global variables declared at the beginning of the file in the line:
  //	int hue_lower, hue_upper, sat_lower, sat_upper, value_lower, value_upper;
  // The default value is used if the specified parameter cannot be retrieved from the parameter server
  // See 0.0.2 of http://wiki.ros.org/roscpp_tutorials/Tutorials/Parameters
  nh_.param<int>("hue_lower", hue_lower, 80);
  nh_.param<int>("hue_upper", hue_upper, 150);
  nh_.param<int>("sat_lower", sat_lower, 20);
  nh_.param<int>("sat_upper", sat_upper, 255);
  nh_.param<int>("value_lower", value_lower, 20);
  nh_.param<int>("value_upper", value_upper, 255);
```
Recall that `hue_lower`, `hue_upper`, `sat_lower`, `sat_upper`, `value_lower`, and `value_upper` were referenced in the `imageCallback` function to define a HSV range to filter for:
```cpp
//Define the range of blue pixels
cv::Scalar lower_thresh(hue_lower, sat_lower, value_lower);
cv::Scalar upper_thresh(hue_upper, sat_upper, value_upper);
```
<a href="#top" class="top" id="compile_vision_tracking"></a>
To try the `vision_tracking` node with dynamically reconfigurable hue, saturation, and value ranges, recompile your code using:
```
$ cd ~/catkin_ws
$ catkin_make
```

Then, launch the `vision_tracking` node:
```
$ roslaunch basic_opencv vision_tracking.launch
```

Then open the image_view in a new terminal:
```
$ rosrun image_view image_view image:=/user/image1
```

Finally, start the dynamic reconfigure GUI:
```
$ rosrun rqt_reconfigure rqt_reconfigure
```

Try updating the values of the hue, value, and saturation in the GUI until you are able to isolate the object.  Start by adjusting only the hue values according to the color of your object.  The list below provides the hue ranges of various colors:

- Red: (0, 15), (160, 180)
- Yellow: (20, 45)
- Green: (50, 75)
- Blue: (100, 135)
- Purple: (140, 160)

Notice that the color red has two ranges because it occurrs where the number wrap around; if your object is red, test each range to see which one detects the object best.  Keep manipulating the parameters until only the object is visible as white on the screen.

### Topic 2: Image Moments and Center of Mass

After computing the pixels that are part of the object, the next step is to find the center of the object.  The image moments will be computed for the `mask` matrix.  These moments will then be used to determine the "center of mass" of the object.

Add the following lines after the `inRange` function call.  You can add a debug statement like `ROS_INFO("%f %f", center_of_mass.x, center_of_mass.y)` to see coordinates of the center of mass.

```cpp
//Calculate moments of mask
moments = cv::moments(mask, true);

//Calculate center of mass using moments
center_of_mass.x = moments.m10 / moments.m00;
center_of_mass.y = moments.m01 / moments.m00;
```

OpenCV has a special datatype called `Moment` that stores the moments of an image.  You can read more about the values stored in a `Moment` on OpenCV's documentation: http://docs.opencv.org/2.4/modules/imgproc/doc/structural_analysis_and_shape_descriptors.html#moments

To visualize the center of mass, add a function that draws a red circle where the center of mass is located.  The OpenCV function to do this is `circle`.  Add the red circle to the `dst` matrix after the initialization of `dst` from `mask`.

Now you can recompile and relaunch the `vision_tracking` node along with the reconfigure GUI and image_view [(refresher on how to do so)](#compile_vision_tracking) .  You should now see a red dot that is located at the center of your object.  This system could be used by robots to find the ball in a game of soccer or locate fruit on a tree for harvesting.  Try using a different colored object to see if you can locate it as well.

# Review
- [What is a catkin workspace?](#quiz_workspace)
- [What is a ROS package?](#quiz_package)
- [How do you build ROS packages?](#quiz_compile)
- Define the following ROS components:
	- [node](#quiz_node)
	- [topic](#quiz_topic)
	- [publisher](#quiz_topic) ([Code Example](#quiz_pub_example))
	- [subscriber](#quiz_sub) ([Code Example](#quiz_sub_example))
	- [master](#quiz_master)
	- [message](#quiz_message)
- [What does a launch file do?](#quiz_launchfile)

Congratulations! You've got the fundamentals down, and are ready to build larger robotics projects using ROS!

<a href="#top" class="top" id="rrp_table_of_contents"></a>
# Choose Your Own Adventure: *ROS Robotics Projects*
One of my favorite things about ROS is that the loose coupling of nodes and the abundance of open-source software makes it possible for beginners to assemble a robot system by plug-and-playing ROS packages for the various functionalities.

The projects from *ROS Robotics Projects* will allow you to experiment with various open-source ROS packages that you may want to incorporate into your own ROS projects.

Here are some recommended projects that don't require extra hardware:
- [Chapter 2: Face Detection using OpenCV](#adventure_face_detection)
- [Chapter 3: Building a Siri-Like Chatbot in ROS](#adventure_chatbot)
- [Chapter 6: Object Detection and Recognition](#adventure_objectdetection)
- [Chapter 7: Deep Learning using ROS and Tensorflow](#adventure_deeplearning)
- [Chapter 8.5: ROS on Android](#adventure_android)

If you have access to an Arduino, here are some tutorials for using ROS with Arduino:
- http://wiki.ros.org/rosserial_arduino/Tutorials

<a href="#top" class="top" id="adventure_face_detection"></a>
##  Chapter 2: Face Detection using OpenCV
### Start reading Chapter 2 [here](http://proquest.safaribooksonline.com.ezproxy.cul.columbia.edu/book/hardware/9781783554713/ros-robotics-projects/ch02_html?uicode=columbia).
**Note: In the book, the Face Detection project is a component of a larger Face Detection and Tracking project using Dynamixel Servos.** The original code for this project has portions that depend on the servo code. We modified the project code so that the face detection portion can bue built and run on its own. This modified version of the code for this project is already in `~/catkin_ws/src/face_tracker_pkg`. Some of the instructions for this project will change because of the modifications.  **Each heading below corresponds to the title of a page in the book, and the text below the heading describes how your project differs from the book's.**
- **Headings with SKIP affixed to them indicate pages that can skipped witout loss of understanding.**
- **Headings with READ & SKIP may be useful to read, but you don't need to follow the instructions on the page.**
- **All other headings indicate pages that should be read, and the instructions within them executed, with possible modification.**

###   Chapter 2. Face Detection and Tracking Using ROS, OpenCV and Dynamixel Servos
- We will not be mounting a USB webcam on an AX-12 Dynamixel servo, nor controlling the servo. However, we will be covering:
	- An overview of the project
	- Software prerequisites
	- Creating ROS packages for a face tracker and controller
	- The ROS-OpenCV interface
	- Implementing a face tracker
### Overview of the project
- There will be no Dynamixel servo tracking component. Our version of the project will involve only one ROS package, for face detection and finding the centroid of the face. This package, called `face_tracking_pkg`, is already inside `~/ros_robotics_projects_ws/src`, so **no source code needs to be re-cloned.**
### Hardware and software prerequisites
- If you were able to complete Level 2, then you've satisfied all of the software prerequisites. (Recognize any of ROS packages in the table? You used `usb_cam` and `cv_bridge` for Level 2 as well!)
### Installing dependent ROS packages - SKIP
### Installing the usb_cam ROS package - READ & SKIP
### Creating a ROS workspace for dependencies - SKIP
- The workspace `~/ros_project_dependencies_ws` is included with the RoboticsTrack VirtualBox image, so you do not need to create a new workspace for keeping project dependencies. The line `source ~/ros_project_dependencies_ws/devel/setup.bash` is already in `~/.bashrc`, which is necessary for environment setup.
### Interfacing Dynamixel with ROS - SKIP
### Creating face tracker ROS packages - READ & SKIP
- The workspace `~/ros_robotics_projects_ws ` came included with the RoboticsTrack VirtualBox image, so you do not need to create a new workspace for keeping ROS project files. The line `source ~/ros_robotics_projects_ws/devel/setup.bash` is already in `~/.bashrc`, which is necessary for environment setup.
- You do not need to git clone the source code of the book from GitHub; `~/ros_robotics_projects_ws/src` already has the packages necessary for the projects mentioned in this tutorial.
- You can read about the dependencies of `face_tracker_pkg`, but you do not need to use `catkin_create_pkg` to create a new `face_tracker_pkg` yourself.
- Please read the section **The interface between ROS and OpenCV**, as a refresher of what we learned in Level 2.
### Working with the face-tracking ROS package
- Note that there will be `face_tracker_control` package
- You can enter the `face_tracker_pkg` by typing `roscd face_tracker_pkg` or going to `~/ros_robots_projects_ws/src/face_tracker_package`
### Understanding the face tracker code
- This page refers to code in `~/ros_robotics_projects_ws/src/face_tracker_package/src/face_tracker_node.cpp`.

### Understanding CMakeLists.txt
 - This page refers to code in `~/ros_robotics_projects_ws/src/face_tracker_package/CMakeLists.txt`
### The track.yaml file
- [Review of ROS Parameters](#quiz_param)
-  You don't need to change any parameters in `track.yaml`
### The launch files - READ & SKIP
- For our version of the project, `start_usb_cam.launch` (full path `~/ros_robotics_projects_ws/src/face_tracker_pkg/launch/start_usb_cam.launch`) contains only the following:
```xml
<launch>
<!-- Launching USB CAM launch file -->
<
```

<a href="#top" class="top" id="adventure_chatbot"></a>
## Chapter 3: Building a Siri-Like Chatbot in ROS
### Start reading Chapter 3 [here](http://proquest.safaribooksonline.com.ezproxy.cul.columbia.edu/book/hardware/9781783554713/ros-robotics-projects/ch03_html?uicode=columbia).

#### Installing PyAIML on Ubuntu 16.04 LTS
Please use 

<a href="#top" class="top" id="adventure_objectdetection"></a>
## Chapter 6: Object Detection and Recognition

<a href="#top" class="top" id="adventure_deeplearning"></a>
## Chapter 7: Deep Learning using ROS and Tensorflow

<a href="#top" class="top" id="adventure_android"></a>
## Chapter 8.5: ROS on Android
