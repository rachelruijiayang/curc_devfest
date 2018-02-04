
<a id="top"></a>
# CURC DevFest Robotics Track
*How to build robots using Robot Opeating System, OpenCV, and other ROS packages.*

Written and developed by [Rachel Yang](https://github.com/rachelruijiayang), [Ignacio Ramirez-Piriz](https://github.com/ir2331), and [Columbia University Robotics Club](http://www.columbia.edu/cu/roboticsclub/). 

Portions of this tutorial are adapted from [NVIDIA's Robotics Teaching Kit with 'Jet'](https://developer.nvidia.com/teaching-kits), [*ROS Robotics Projects* by Lentin Joseph](https://www.amazon.com/ROS-Robotics-Projects-Lentin-Joseph/dp/1783554711) (available to Columbia students through [CLIO](https://clio.columbia.edu/quicksearch?q=ros+robotics+projects)), and [*Effective Robotics Programming with ROS - Third Edition* by Anil Mahtani and Luis Sanchez](https://www.amazon.com/Effective-Robotics-Programming-ROS-Third/dp/1786463652) (also available through [CLIO](https://clio.columbia.edu/quicksearch?q=effective+robotics+programming+with+ros+third+edition&commit=Search)).

## What We're Building
Throughout this tutorial, we're going to build a ROS system that uses a webcam and computer vision to *sense* the location of the user's face and then *actuate* a simulated turtle to follow the user's face.

This document advances linearly, meaning that level 2 builds atop level 1, level 3 atop level 2, and so on. Each level after level 1 has its own branch in the GitHub repository for this tutorial. If you don't want to do each level in order, you can `git checkout` the branch for the level you would like to skip to.

## Why learn ROS?
ROS is an open-source framework for building robots. 

A robot is just an system in which sensors (such as cameras) connect to processors, which connect to actuators to make the robot move (or take some other action). ROS provides a publish-subscribe messaging infrastructure that makes it easier for different parts of a robot system to talk to each other. ROS also provides an extensive set of tools for configuring, starting, debugging, simulating, testing, and stopping robot systems. Finally, ROS provides a broad collection of open-source libraries that implement useful robot functionality, with a focus on mobility, manipulation, and perception, and a large active developer community. [[Source]](https://answers.ros.org/question/12230/what-is-ros-exactly-middleware-framework-operating-system/)

[Here are some past robots that have been built using ROS.](https://www.youtube.com/watch?v=Z70_3wMFO24)

## Prerequisites

 - At least 25 GB of free storage space on your computer
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

### Importing the curc_devfest Image
Download **curc_devfest.ova** from [this link](www.google.com). An .ova file describes a virtual machine.
Then, inside Oracle VM VirtualBox Manager, select File > Import Appliance, click on the folder icon, and find **curc_devfest.ova**. 

Review the appliance settings. Because my computer has 12 GB of RAM, I selected 8192 MB (8 GB) of RAM as the default amount for **curc_devfest**.
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
$ source ~/catkin_ws/devel/setup.bash
```
to add our workspace into the PATH that ROS uses to search for executables. Finally, run
```
$ source ~/.bashrc
```
to reload your `bash` settings.

<a href="#top" class="top" id="level1"></a>
# Level 1: Intro to ROS
In this section, you will learn the structure and fundamental building blocks of ROS. Then, using those concepts, you'll write some C++ ROS code to programmatically control a simulated turtle. 

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
`turtlesim` is a ROS package that implements a simulated robot turtle. The turtle's position on the screen will change based on input velocity commands. We will practice what we have learned by using `turtlesim` as an example.
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
Your new path will be `/opt/ros/kinetic/share/turtlesim`. 

### Playing with ROS Nodes
Before starting this tutorial, it is important that you open a terminal and start `roscore` using the following command:
```
$ roscore
```
`roscore` is a collection of nodes and programs that are pre-requisites of a ROS-based system. You must have a roscore running in order for ROS nodes to communicate. `roscore` will start up:
- a ROS Master- a ROS Parameter Server
- a rosout logging node
Now that `roscore` is running, we are going to use `rosnode list` to list all the nodes running. Open up a new terminal (while keeping the terminal running `roscore` alive) and run:
```
$ rosnode list
```
- You will see:
	```
	/rosout
	```
This showed us that there is only one node running: `rosout`. This is always running as it collects and logs nodes' debugging output.
### Using `rosrun`
`rosrun` allows you to use the package name to directly run a node within a package (without having to know the package path).
Usage:
```
$ rosrun [package_name] [node_name]
```
So now we can run the turtlesim_node in the turtlesim package.
Then, in a new terminal:
```
$ rosrun turtlesim turtlesim_node
```
You will see the `turtlesim` window:![enter image description here](http://wiki.ros.org/ROS/Tutorials/UnderstandingNodes?action=AttachFile&do=get&target=turtlesim.png)

In a new terminal:
```
$ rosnode list
```
You will now see that two nodes are running:
```
/rosout
/turtlesim
```

### Understanding Topics
You can drive the turtle aruond its window by running in a new terminal:
```
rosrun turtlesim turtle_telop_key
```
Now you can use the arrow (up, down, left, right) keys of the keyboard to drive the turtle around. If you can not drive the turtle **select the terminal window of the `turtle_teleop_key`** to make sure that the keys that you type are recorded.
![enter image description here](http://wiki.ros.org/ROS/Tutorials/UnderstandingTopics?action=AttachFile&do=get&target=turtle_key.png)

The `turtlesim` node and the `turtle_teleop_key` node are communicating with each other over a ROS topic. `turtle_teleop_key` converts input keystrokes into velocity commands for the turtle. For example, the up key is converted into a command to move with some velocity in the up direction. It publishes those velocity commands on a topic, while `turtlesim` subscribes to the same topic to receive the velocity commands.

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
!!!CHANGE THE IMAGE!!!
![enter image description here](http://wiki.ros.org/ROS/Tutorials/UnderstandingTopics?action=AttachFile&do=get&target=rqt_graph_turtle_key.png)

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
$ $ rostopic pub /turtle1/cmd_vel  geometry_msgs/Twist -r 1 -- "linear:
x: 1.0
y: 0.0
z: 0.0
angular:
x: 0.0
y: 0.0
z: 1.0"
```
You will see the turtle doing a curve, as shown:
![enter image description here](http://wiki.ros.org/ROS/Tutorials/UnderstandingTopics?action=AttachFile&do=get&target=turtle%28rostopicpub%29.png)

## Your Own Turtlesim Controller


### Creating a ROS package
We will create a new package in our workspace `catkin_ws`. `catkin_create_pkg` is a command-line tool that automates some of the package-creation process.
```
$ cd ~/dev/catkin_ws/src
$ catkin_create_pkg turtle_control_basic std_msgs roscpp
```
This will create a `turtle_control_basic` folder which contains a package.xml and a CMakeLists.txt, which have been partially filled out with the information you gave `catkin_create_pkg`.
`catkin_create_pkg` requires that you give it a package_name and optionally a list of dependencies on which that package depends:

The following dependencies are included:
- [`std_msgs`](http://wiki.ros.org/std_msgs): This contains common message types representing primitive data types (e.g. `bool`, `int64`, `float64`, `uint8`) and other basic message constructs.- [`roscpp`](http://wiki.ros.org/roscpp): This is a C++ implementation of ROS. `roscpp` is the most widely used ROS client library and is designed to be the high-performance library for ROS. ROS also has a Python API, but we do not use it in this tutorial.

Now, let's add code to the`turtle_control_basic` package to programmatically control the `turtlesim` turtle.simulated turtle whose position on the screen will change based on input velocity commands.


### Navigating through the ROS filesystem


## ROS Tools
### rqt_graph

# Level 2: Basic OpenCV
To skip to the start of Level 2: `git checkout level2`
# Level 3: Using CV Output to Actuate the Turtle
To skip to the start of Level 3: `git checkout level3`
# Level 4: Face Detection using OpenCV
To skip to the start of Level 4: `git checkout level4`
# Level5: What’s next?
To skip to the start of Level 5: `git checkout level5`
