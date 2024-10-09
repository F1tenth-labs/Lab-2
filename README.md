# Lab 2: Automatic Emergency Braking
## Prerequisites
- [Lab 1 - Introduction to ROS2](https://f1tenth-coursekit.readthedocs.io/en/latest/assignments/labs/lab1.html)
- [Lecture 2 - Automatic Emergency Braking](https://f1tenth-coursekit.readthedocs.io/en/latest/lectures/ModuleA/lecture02.html)
- [Tutorial 2 - Introduction to F1TENTH Simulator](https://f1tenth-coursekit.readthedocs.io/en/latest/lectures/ModuleA/tutorial2.html)
  - Is your simulator working and can you drive the car with your keyboard
## Process
1. Setup: Clone the f1tenth_lab2_template repo into your sim workspace
2. Implement: code the safety node
3. Test: Build the workspace and run the simulation and safety node
## Setup
First move into your simulators workspace
```BASH
cd ~/sim_ws/src
```
Then clone the repo into your workspace
```BASH
git clone https://github.com/f1tenth/f1tenth_lab2_template.git
```
Your current directory should look similar to this
```
~/sim_ws/src
├── f1tenth_gym_ros
│   ├── ...
└── f1tenth_lab2_template
    ├── LICENSE
    ├── README.md
    ├── safety_node
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   ├── safety_node
    │   │   └── __init__.py
    │   ├── scripts
    │   │   └── safety_node.py
    │   ├── setup.cfg
    │   ├── setup.py
    │   └── src
    │       └── safety_node.cpp
    └── SUBMISSION.md
```
## Implement
### Get the necessary prerequiste information
#### Learn about the topics
We first want to know about the topics we are publishing to and subcribing from.
After **running the simulator in a seperate window** we can try listing all the topics.
```BASH
○ → ros2 topic list
/clicked_point
/clock
/cmd_vel
/drive
/ego_racecar/odom
/ego_robot_description
/goal_pose
/initialpose
/joint_states
/map
/map_server/transition_event
/map_updates
/parameter_events
/rosout
/scan
/tf
/tf_static
```
`/drive`, `/scan`, and `/ego_racecar/odom` are the topics we will be interacting with in todays lab.

Lets get some more information on these topics:
#### /drive
```BASH
○ → ros2 topic info /drive
Type: ackermann_msgs/msg/AckermannDriveStamped
Publisher count: 0
Subscription count: 1
```
We will be publishing to this node, so its a good idea to understand what type of message we should be sending to this node. We can get more information on `ackermann_msgs/msg/AckermannDriveStamped` using the `ros2 interface show`
```BASH
○ → ros2 interface show ackermann_msgs/msg/AckermannDriveStamped
## Time stamped drive command for robots with Ackermann steering.
#  $Id$

std_msgs/Header header
AckermannDrive  drive
```
So this node has a field for an AckermannDrive message aswell. Lets use that command again to find information on `AckermannDrive`
```BASH
○ → ros2 interface show ackermann_msgs/msg/AckermannDrive
## Driving command for a car-like vehicle using Ackermann steering.
#  $Id$

...

float32 steering_angle          # desired virtual angle (radians)
float32 steering_angle_velocity # desired rate of change (radians/s)

...

float32 speed                   # desired forward speed (m/s)
float32 acceleration            # desired acceleration (m/s^2)
float32 jerk                    # desired jerk (m/s^3)
```
Now we have a detailed view at all the fields of the `AckermannDrive` and `AckermannDriveStamped` messages, we are able to create our own when necessary.

### Implement the node
After we have the necessary information, we can begin to create our node.
The first thing we are going to do in our `safety_node.py` is initialize the publishers and subscribers
#### Publisher init
Inside of our `def __init__(self)` we are going to create our publisher
```PY
  self.drive_pub = self.create_publisher(
     AckermannDriveStamped,
     '/drive',
     10
  )
```
#### Subscriber init
After our publisher, we initialize our subscribers
```PY
  self.scan_sub = self.create_subscription(
    LaserScan,
    '/scan',
    self.scan_callback,
    10
  )
  self.odom_sub = self.create_subscription(
    Odometry,
    '/ego_racecar/odom',
    self.odom_callback,
    10
  )
```
