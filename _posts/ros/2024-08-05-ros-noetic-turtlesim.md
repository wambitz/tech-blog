---
layout: post
title: "Using ROS Noetic Turtle Sim on Ubuntu 20.04"
categories: ROS, Noetic, TurtleSim, Ubuntu
---

## Using ROS Noetic Turtle Sim on Ubuntu 20.04

In the following sections, we will investigate Turtle Sim communication commands and will cover the following steps:

- Listing all active nodes
- Listing all topics
- Getting information about topics
- Showing message information
- Echoing messages in real-time

### Prerequisites

Install ROS Noetic on Ubuntu 20.04. Follow the official page instructions for installation: [ROS Noetic Installation](https://wiki.ros.org/noetic/Installation/Ubuntu).

### Start Roscore

This step is necessary before launching turtlesim or any other turtlesim command.

```bash
roscore
```

![Roscore command output]({{ site.baseurl }}/assets/images/posts/2024-08-05-ros-noetic-turtlesim/roscore-command-output.png)


### Turtlesim Launch

Each one of these commands needs to run in a new Terminal instance or a new tab.

#### Start TurtleSim Node

Starts the turtlesim GUI:

```bash
rosrun turtlesim turtlesim_node
```
![rosrun turtlesim_node output]({{ site.baseurl }}/assets/images/posts/2024-08-05-ros-noetic-turtlesim/rosrun-turtlesim-node-output.png)

#### Start TurtleSim Teleop Key

Use the keyboard to manipulate the turtle robot:

```bash
rosrun turtlesim turtle_teleop_key
```

![rosrun turtle_teleop_key output]({{ site.baseurl }}/assets/images/posts/2024-08-05-ros-noetic-turtlesim/rosrun-turtle-teleop-key-output.png)

### Listing all Active Nodes

Now that we’ve launched `turtlesim_node` and played around with sending commands via the `turtle_teleop_key` node, to get a list of all nodes that are active and have been registered with the ROS Master, we can use the command `rosnode list`. Let’s do so now:

```bash
rosnode list
```

![rosnode list output]({{ site.baseurl }}/assets/images/posts/2024-08-05-ros-noetic-turtlesim/rosnode-list-output.png)

- `/rosout`: This node is launched by `roscore`. It subscribes to the standard `/rosout` topic, the topic to which all nodes send log messages.
- `/teleop_turtle`: This is our keyboard teleop node. Notice that it’s not named `turtle_teleop_key`. There’s no requirement that a node’s broadcasted name is the same as the name of its associated executable.
- `/turtlesim`: The node name associated with the turtlebot_sim node.

### Listing all Active Topics

In a similar fashion, we are able to query the ROS Master for a list of all topics. To do so, we use the command `rostopic list`.

```bash
rostopic list
```

![rostopic list output]({{ site.baseurl }}/assets/images/posts/2024-08-05-ros-noetic-turtlesim/rostopic-list-output.png)

- `/rosout_agg`: Aggregated feed of messages published to `/rosout`.
- `/turtle1/cmd_vel`: Topic on which velocity commands are sent/received. Publishing a velocity message to this topic will command `turtle1` to move.
- `/turtle1/color_sensor`: Each turtle in turtlesim is equipped with a color sensor, and readings from the sensor are published to this topic.
- `/turtle1/pose`: The position and orientation of `turtle1` are published to this topic.

### Get Information About a Specific Topic

If we wish to get information about a specific topic, such as who is publishing to it, subscribed to it, or the type of message associated with it, we can use the command `rostopic info`. Let’s check the `/turtle1/cmd_vel` topic:

```bash
rostopic info /turtle1/cmd_vel
```

![rostopic info output]({{ site.baseurl }}/assets/images/posts/2024-08-05-ros-noetic-turtlesim/rostopic-info-output.png)

As expected, there are two nodes registered on this topic: one publisher (the `teleop_turtle` node) and one subscriber (the `turtlesim` node). The type of message used on this topic is `geometry_msgs/Twist`.

### Show Message Information

Let’s get some more information about the `geometry_msgs/Twist` message on the `/turtle1/cmd_vel` topic. To do so, we will use the `rosmsg info` command:

```bash
rosmsg info geometry_msgs/Twist
```

![rosmsg info output]({{ site.baseurl }}/assets/images/posts/2024-08-05-ros-noetic-turtlesim/rosmsg-info-output.png)

We can see that a `Twist` message consists of two `Vector3` messages: one for linear velocity and another for angular velocity, with each velocity component (`x`, `y`, `z`) represented by a `float64`.

**Note**: Sometimes the message definition doesn’t provide an ample amount of detail about a message type. For instance, how can we be sure that linear and angular vectors above refer to velocities, and not positions? One way to get more detail would be to look at the comments in the message’s definition file. To do so, we can issue the following command:

```bash
rosed geometry_msgs Twist.msg
```

### Echo Messages on a Topic

Sometimes it may be useful to look at a topic’s published messages in real-time. To do so, we can use the `rostopic echo` command. Let’s take a look at the `/turtle1/cmd_vel` topic:

```bash
rostopic echo /turtle1/cmd_vel
```

![rostopic echo output]({{ site.baseurl }}/assets/images/posts/2024-08-05-ros-noetic-turtlesim/rosmsg-info-output.png)

If we then command the turtle to move from the `turtle_teleop_key` window, we will be able to see the output message in real-time!
