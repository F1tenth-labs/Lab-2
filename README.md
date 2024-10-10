# Lab 2: Automatic Emergency Braking
## Prerequisites
- [Lab 1 - Introduction to ROS2](https://f1tenth-coursekit.readthedocs.io/en/latest/assignments/labs/lab1.html)
- [Lecture 2 - Automatic Emergency Braking](https://f1tenth-coursekit.readthedocs.io/en/latest/lectures/ModuleA/lecture02.html)
- [Tutorial 2 - Introduction to F1TENTH Simulator](https://f1tenth-coursekit.readthedocs.io/en/latest/lectures/ModuleA/tutorial2.html)
  - Is your simulator working and can you drive the car with your keyboard
## Process
1. [Setup](#setup): Clone the f1tenth_lab2_template repo into your sim workspace
2. [Implement](#implement): code the safety node
3. [Test](#test): Build the workspace and run the simulation and safety node
4. [Conclusion](#conclusion): Potential problems with our implementation and future improvements
## Setup
First move into your simulators workspace
```BASH
cd ~/sim_ws/src
```
Then clone the repo into your workspace
```BASH
git clone https://github.com/f1tenth/f1tenth_lab2_template.git
```
The repo did not work exactly out of the box so there is some moving around we have to do before we can jump into it.

First, optionally, we can remove the .md files with 
```
rm ~/sim_ws/src/f1tenth_lab2_template/*.md
```
### Python Only:
We have to move the `safety_node.py` into the `~/sim_ws/src/f1tenth_lab2_template/safety_node/safety_node` directory and delete the unused scripts directory
```BASH
mv ~/sim_ws/src/f1tenth_lab2_template/safety_node/scripts/safety_node.py ~/sim_ws/src/f1tenth_lab2_template/safety_node/safety_node/safety_node.py
rm -f ~/sim_ws/src/f1tenth_lab2_template/safety_node/scripts/
```
Remove unecessary c++ files
```BASH
rm -rf ~/sim_ws/src/f1tenth_lab2_template/safety_node/src/
rm -f ~/sim_ws/src/f1tenth_lab2_template/safety_node/CMakeLists.txt
```
Create a resource directory
```BASH
mkdir ~/sim_ws/src/f1tenth_lab2_template/safety_node/resource
touch ~/sim_ws/src/f1tenth_lab2_template/safety_node/resource/safety_node
```
Your current directory should look similar to this:
```
~/sim_ws/src
├── f1tenth_gym_ros
│   ├── ...
└── f1tenth_lab2_template
    ├── LICENSE
    └── safety_node
        ├── package.xml
        ├── resource
        │   └── safety_node
        ├── safety_node
        │   ├── __init__.py
        │   └── safety_node.py
        ├── setup.cfg
        └── setup.py
```
Now we have to edit the package.xml and setup.py:
#### package.xml
Remove `<buildtool_depend>ament_cmake</buildtool_depend>` and `<depend>rclcpp</depend>`

Replace `<build_type>ament_cmake</build_type>` in the `<export>` tag with `<build_type>ament_cmake_python</build_type>`

Optionally: update your maintainer, maintainer email, and description

#### setup.py
If you updated your maintainer and such, copy that information to this file aswell

### C++ Only:
Remove unnecessary python files
```BASH
rm -rf ~/sim_ws/src/f1tenth_lab2_template/safety_node/safety_node
rm -rf ~/sim_ws/src/f1tenth_lab2_template/safety_node/scripts
rm -f rm -rf ~/sim_ws/src/f1tenth_lab2_template/safety_node/setup.*
```
Now we have to create a header file
```BASH
mkdir -p ~/sim_ws/src/f1tenth_lab2_template/safety_node/include/safety_node
touch ~/sim_ws/src/f1tenth_lab2_template/safety_node/include/safety_node/safety_node.hpp
```
I wont show you how to make a header file for your c++ code, however you can just look in the repo and you can find it

Now your package should look like this:
```
~/sim_ws/src
├── f1tenth_gym_ros
│   ├── ...
└── f1tenth_lab2_template
    ├── LICENSE
    └── safety_node
        ├── CMakeLists.txt
        ├── include
        │   └── safety_node
        │       └── safety_node.hpp
        ├── package.xml
        └── src
            └── safety_node.cpp
```

Now we have to edit the package.xml:
#### package.xml
Remove `<buildtool_depend>ament_cmake_python</buildtool_depend>` and `<depend>rclpy</depend>`

Optionally: update your maintainer, maintainer email, and description

### Both Python and C++:
Move the `safety_node.py` into the `~/sim_ws/src/f1tenth_lab2_template/safety_node/safety_node` directory and delete the unused scripts directory

(above in py)

Create a resource directory

(above in py)

Create a header file

(above in c++)


Your tree should look similar to this
```
~/sim_ws/src
├── f1tenth_gym_ros
│   ├── ...
└── f1tenth_lab2_template
    ├── LICENSE
    └── safety_node
        ├── CMakeLists.txt
        ├── include
        │   └── safety_node
        │       └── safety_node.hpp
        ├── package.xml
        ├── resource
        │   └── safety_node
        ├── safety_node
        │   ├── __init__.py
        │   └── safety_node.py
        ├── setup.cfg
        ├── setup.py
        └── src
            └── safety_node.cpp
```

Update CMakeLists.txt
Replace:
```
# Create Cpp executable
add_executable(safety_node src/safety_node.cpp)
ament_target_dependencies(safety_node 
  rclcpp geometry_msgs ackermann_msgs nav_msgs sensor_msgs std_msgs
)

# Install Cpp executables
install(TARGETS
  safety_node
  DESTINATION lib/${PROJECT_NAME})
...
# Install Python executables
install(PROGRAMS
  scripts/safety_node.py
  DESTINATION lib/${PROJECT_NAME}
```
With:
```
# Create Cpp executable
add_executable(safety_node_cpp src/safety_node.cpp)
ament_target_dependencies(safety_node_cpp 
  rclcpp geometry_msgs ackermann_msgs nav_msgs sensor_msgs std_msgs
)

# Install Cpp executables
install(TARGETS
  safety_node_cpp
  DESTINATION lib/${PROJECT_NAME})
...
# Install Python executables
install(PROGRAMS
  safety_node/safety_node.py
  DESTINATION lib/${PROJECT_NAME}
)
```
Update setup.py and package.xml
#### setup.py
Inside of `entry_points`, replace `safety_node = safety_node.safety_node:main` with `safety_node_py = safety_node.safety_node:main`

Optionally: update your maintainer, maintainer email, and description

#### package.xml
If you updated your maintainer and such, copy that information to this file aswell

We just changed the names of our executables so when we run our nodes we can choose to run either c++ or python

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
We will be publishing to this node, so its a good idea to understand what type of message we should be sending to this node. We can get more information on `ackermann_msgs/msg/AckermannDriveStamped` using the `ros2 interface show` command.

NOTE: If you're wondering what is subcribed to this node, you can check yourself using `ros2 topic info /drive --verbose`
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

We will be publishing to the `/drive` topic to set the `speed` to 0 in order to stop the car when we choose. As you can see, `speed` is a field in `AckermannDrive`. This means when we create an `AckermannDriveStamped` object in python, we can access the speed like so:
```PY
brake_msg = AckermannDriveStamped()
brake_msg.drive.speed = 0.0
```
By getting information like this from other topics, we will be able to analyze the data given to us and send data in the correct format

<details>
  <summary><h3>/scan</h3></summary>
  Publishes LIDAR data in LaserScan messages, providing information about the distance to nearby obstacles. It's used for obstacle detection, collision avoidance, and environmental mapping.
</details>

```BASH
○ → ros2 topic info /scan
Type: sensor_msgs/msg/LaserScan
Publisher count: 1
Subscription count: 1
```
<details>
  <summary><h5>LaserScan</h5></summary>
  Provides range data from LIDAR sensors, containing distances to obstacles around the vehicle, along with angle and intensity information for each scan point.
</details>


```BASH
○ → ros2 interface show sensor_msgs/msg/LaserScan
# Single scan from a planar laser range-finder
#
# If you have another ranging device with different behavior (e.g. a sonar
# array), please find or create a different message, since applications
# will make fairly laser-specific assumptions about this data

std_msgs/Header header # timestamp in the header is the acquisition time of
                             # the first ray in the scan.
                             #
                             # in frame frame_id, angles are measured around
                             # the positive Z axis (counterclockwise, if Z is up)
                             # with zero angle being forward along the x axis

float32 angle_min            # start angle of the scan [rad]
float32 angle_max            # end angle of the scan [rad]
float32 angle_increment      # angular distance between measurements [rad]

float32 time_increment       # time between measurements [seconds] - if your scanner
                             # is moving, this will be used in interpolating position
                             # of 3d points
float32 scan_time            # time between scans [seconds]

float32 range_min            # minimum range value [m]
float32 range_max            # maximum range value [m]

float32[] ranges             # range data [m]
                             # (Note: values < range_min or > range_max should be discarded)
float32[] intensities        # intensity data [device-specific units].  If your
                             # device does not provide intensities, please leave
                             # the array empty.
```

<details>
  <summary><h3>/ego_racecar/odom</h3></summary>
  Provides odometry information for the ego racecar, including its position and velocity. It tracks the car’s movement in the environment, helping with localization, navigation, and motion control.
</details>

```BASH
○ → ros2 topic info /ego_racecar/odom 
Type: nav_msgs/msg/Odometry
Publisher count: 1
Subscription count: 0
```
<details>
  <summary><h5>Odometry</h5></summary>
  Provides information about the vehicle's position, orientation, and velocity over time, allowing tracking of its movement in the environment.
</details>

```BASH
○ → ros2 interface show nav_msgs/msg/Odometry
# This represents an estimate of a position and velocity in free space.
# The pose in this message should be specified in the coordinate frame given by header.frame_id
# The twist in this message should be specified in the coordinate frame given by the child_frame_id

# Includes the frame id of the pose parent.
std_msgs/Header header

# Frame id the pose points to. The twist is in this coordinate frame.
string child_frame_id

# Estimated pose that is typically relative to a fixed world frame.
geometry_msgs/PoseWithCovariance pose

# Estimated linear and angular velocity relative to child_frame_id.
geometry_msgs/TwistWithCovariance twist
```
<details>
  <summary><h5>PoseWithCovariance</h5></summary>
  Combines position, orientation, and the associated uncertainty (covariance matrix) of the pose, offering more detailed information for localization and mapping.
</details>

```BASH
○ → ros2 interface show geometry_msgs/msg/PoseWithCovariance
# This represents a pose in free space with uncertainty.

Pose pose

# Row-major representation of the 6x6 covariance matrix
# The orientation parameters use a fixed-axis representation.
# In order, the parameters are:
# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
float64[36] covariance
```
<details>
  <summary><h5>Pose</h5></summary>
  Represents the position (x, y, z) and orientation (as a quaternion) of an object in 3D space.
</details>

```BASH
○ → ros2 interface show geometry_msgs/msg/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
```
<details>
  <summary><h5>Point</h5></summary>
  Describes a 3D point in space with x, y, and z coordinates.
</details>

```BASH
○ → ros2 interface show geometry_msgs/msg/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
```
<details>
  <summary><h5>Quaternion</h5></summary>
  Represents orientation in 3D space, providing a stable way to describe rotations compared to Euler angles.
</details>

```BASH
○ → ros2 interface show geometry_msgs/msg/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1
```
<details>
  <summary><h5>TwistWithCovariance</h5></summary>
  Combines linear and angular velocity information along with the uncertainty (covariance matrix) for the vehicle’s motion.
</details>

```BASH
○ → ros2 interface show geometry_msgs/msg/TwistWithCovariance
# This expresses velocity in free space with uncertainty.

Twist twist

# Row-major representation of the 6x6 covariance matrix
# The orientation parameters use a fixed-axis representation.
# In order, the parameters are:
# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
float64[36] covariance
```
<details>
  <summary><h5>Twist</h5></summary>
  Contains the linear and angular velocity of a body, used to represent motion in both translation and rotation.
</details>

```BASH
○ → ros2 interface show geometry_msgs/msg/Twist
# This expresses velocity in free space broken into its linear and angular parts.

Vector3  linear
Vector3  angular
```
<details>
  <summary><h5>Vector3</h5></summary>
  Represents a vector in 3D space, commonly used to describe velocity or force, with x, y, and z components.
</details>

```
○ → ros2 interface show geometry_msgs/msg/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
```

We will be using `Twist`'s `Vector3 linear`'s `float64 x` which represents the forward facing velocity of the car.
We will also be using `LaserScans`'s `range_min/max`, `angle_min/max`, and `ranges[]` to calculate the distance from objects and determine what data we should use.

### Implement the node
NOTE: I will only be explaining in python. The c++ code is in the repo however this guide will be strictly in python.

After we have the necessary information, we can begin to create our node.
The first thing we are going to do in our `safety_node.py` is initialize the publishers and subscribers
#### Publisher init
Inside of our `def __init__(self)` we are going to create our publisher. We want to publish an `AckermannDriveStamped` msg to the `/drive` topic
```PY
  self.drive_pub = self.create_publisher(
     AckermannDriveStamped,
     '/drive',
     10
  )
```
#### Subscriber init
After our publisher, we initialize our subscribers
We want to get a `LaserScan` msg from the `/scan` topic and a `Odometery` msg from the `/ego_racecar/odom` topic
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

#### Breaking
When we are about to collide with the wall, we want to tell the car to break. We can do this by sending an `AckermannDriveStamped` message to `/drive` with 0 speed. We will also enter a steering angle of 0.0, which will align the wheels to be straight (this will not change the direction the car is facing).
```PY
def publish_brake(self):
  # Set speed to 0 and align the wheels to be straight
  brake_msg = AckermannDriveStamped()
  brake_msg.drive.speed = 0.0
  brake_msg.drive.steering_angle = 0.0

  self.drive_publisher.publish(brake_msg)
  self.get_logger().info('Emergency Brake has activated!')
```

#### Get current speed
We can get the current speed of the car from the `/ego_racecar/odom/` topic. The Twist message's fields are defined relative to the car's own coordinate frame. This means the we can get how fast the car is moving straight forward by taking the x of the linear field of twist.
```PY
def odom_callback(self, odom_msg):
  # update current speed
  self.speed = odom_msg.twist.twist.linear.x
  self.get_logger().debug(f'Current speed: {self.speed:.2f} m/s')
```

#### Understanding and extracting data from the LaserScan
Inside of our `scan_callback` we want to get the LiDAR data and process it. We want to get the ranges from the LiDAR scanner so we can see how far our car is from any object. These values are stored in the array `ranges`. Lets first understand what this array means.

Each value in `ranges` corresponds to a specific angle from the sensor's origin, with the first value corresponding to the `angle_min` and the last value to `angle_max`.

A value represents the distance to the nearest obstacle along that beam or angle. If the beam does not detect an obstacle, the value may be `inf`, indicating no obstacle within the sensor's maximum range.

Invalid or out of range measurements may be represented as `NaN`.

The length of the `ranges` array depends on the LIDAR's configuration and the number of laser beams or points it scans per revolution. For example, if the LIDAR scans 360 degrees with 1-degree resolution, the `ranges` array will have 360 values.

`ranges`'s data points are all in **meters**.

Now that we understand our input data we can start by extracting the necessary data.

We are using numpy arrays for easier data processing
```PY
ranges = np.array(scan_msg.ranges)
angle_min = scan_msg.angle_min
angle_max = scan_msg.angle_max
num_ranges = len(ranges)
```

We will also create an array of all the angles of the laser. To understand how we are going to do this, we first need to fully understand what the data is.

Imagine a LiDAR scanner that has an `angle_min = -90°`, an `angle_max = 90°`, and a `num_ranges = 5`.

Here is that LiDAR scanner:
```
      -90°        -45°       0°         45°         90°
        |          |          |          |          |
        \          \          |          /          /
         \          \         |         /          /
          \          \        |        /          /
           \          \       |       /          /
            \          \      |      /          /
             \          \     |     /          /
              \          \    |    /          /
               \          \   |   /          /
                \          \  |  /          /
                 \          \ | /          /
                  \__________\|/__________/
                            LIDAR
```
We want to create an array that stores all of the angles. For this example the array would be [-90, -45, 0, 45, 90]. We can easily do this with `NumPy` arrays using the `np.linspace()` which returns an array of numbers that are linearly spaced between two given values (`angle_min` and `angle_max`), with a specified number of elements.
```PY
angles = np.linspace(angle_min, angle_max, num_ranges)
```
#### Creating usable data
We want to filter out all `inf` and `NaN` and any other possible invalid values that would break our code. `NumPy` has another function `np.isfinite()` that takes an array and returns a boolean array where indicies that are `true` correspond to finite values. Using this we can perform boolean indexing and obtain usable data.
```PY
        valid_indices = np.isfinite(ranges)
        ranges = ranges[valid_indices]
        angles = angles[valid_indices]

        if len(ranges) == 0:
            self.get_logger().warn('No valid range data received.')
            return
```
Now we have an array with all the finite distances from the LiDAR and another array with angles correspoding to all of those distances.

#### Calculate the closing velocity
We want to find how fast our car is moving towards a certain object. Given the cars speed and its angle to the object, we can use trig to calculate the closing velocity. Here is a diagram to demonstrate the premise.
```
            ....../   <-- My bad drawing of a moving forward
      ...../
 \.../  θ->|\
           | \
           |  \ 
           |   \ v (direction of movement)
v_closing  |    \
(adj side) |     \
           |      \
           |_______\
    Object^    
```
We want to find `v_closing`, which is the component of the velocity vector that is pointed at the object. Using basic trig we can determine that `cosθ = v_closing/v` or `v_closing = cosθ * v`. We can make an array of all the `v_closing`s using `np.cos()` on our `angles` array.
```PY
v_closing = self.speed * np.cos(angles)
```
`self.speed` is in **m/s** so `v_closing` is aswell
#### Calculate the iTTC
We know `speed = distance/time` which means `time = distance / speed`. We have our distances in our `ranges` array and our speeds in our `v_closing` array. Now we just need our time. `ranges` is in **meters** and `v_closing` is in **m/s**, so our result will be in **seconds** However, we want to remove any possible unneccesary data. In this case, negative `v_closing` is unncessary as we don't care about objects we are moving away from. In our implementation we will have the default value of our `ittc` array be `inf` as to represent that there is not imminent collision.
```PY
ittc = np.full_like(ranges, np.inf)
positive_closing = v_closing > 0
ittc[positive_closing] = ranges[positive_closing] / v_closing[positive_closing]
ittc = np.where(np.isfinite(ittc), ittc, np.inf)
```
Breakdown:

`np.full_like()` makes an np array the same shape (nxm) as `ranges` and sets all values to `np.inf`

`positive_closing` is a boolean mask so only indicies with positive `v_closing`s are used

`ittc = ranges / v_closing` for only the indicies specificed by the boolean mask

`ittc` for each index contains the time it will take for the car to collide with the object along the angle corresponding to the `angles` array

the last line replaces all non-finite values within `ittc` with `np.inf` and keeps all finite values the same

#### Determine if a collision is imminent
We can set our own threshold to determine if a collision is imminent. Back in our `__init__` we can set our iTTC threshold (in **sec** as our ittc is in **sec**)
```PY
self.threshold_iTTC = 1.0
```
Now using this, we can determine if there are any imminent collisions by checking the time for each detected object
```PY
collision_imminent = np.any(ittc < self.threshold_iTTC)
```
If `collision_imminent` is `true`, then we have an object that our car will run into in `ittc (thats under our threshold)` time if nothing is done.

Notice how I said, "if nothing is done". There is still time for the car to move out of the way, so we cant step on the breaks, otherwise we would be sending a false positive. We don't want to step on the breaks if we don't have to.

#### Debounce
A strategy we can implement is debouncing. If a collision is detected (`collision_imminent` is `true`), you increment a counter (`self.debounce_count`). The `debounce_count` tracks how many consecutive times the collision detection logic has reported an imminent collision. Only once `debounce_count` reaches a threshold (`self.debounce_threshold`) does the car trigger the emergency brake.

Back in our `__init__` we can set our debounce threshold and init our debounce counter
```PY
self.debounce_count = 0
self.debounce_threshold = 3
```
Now using simple if-else statements we can increment our `debounce_count` until its over the threshold or until we are out of danger.
```PY
if collision_imminent:
    self.debounce_count += 1
    self.get_logger().warn(f'Collision imminent! iTTC below threshold. Count: {self.debounce_count}')
    if self.debounce_count >= self.debounce_threshold:
        self.publish_brake()
        self.debounce_count = 0  # Reset debounce count after braking
else:
    if self.debounce_count > 0:
        self.debounce_count -= 1  # Decrement debounce count if conditions are safe
    self.get_logger().info('Safe: No imminent collision detected.')
```
Now we have finished our Emergency Braking logic and we can finally test it.
## Test
### Rebuilding
We first have to rebuild our workspace
```BASH
cd ~/sim_ws
colcon build
source install/setup.bash
```
### Running the simulator
In our **first terminal**:
```BASH
source /opt/ros/foxy/setup.bash
source ~/sim_ws/install/local_setup.bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```
### Running teleop_twist:
In our **second terminal**:
```BASH
source /opt/ros/foxy/setup.bash
source ~/sim_ws/install/local_setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
### Running our safety_node:
In our **third terminal**:
```BASH
source /opt/ros/foxy/setup.bash
source ~/sim_ws/install/local_setup.bash
ros2 run safety_node safety_node
```
If you have both C++ and Python compatibility you can either run either of these
```BASH
ros2 run safety_node safety_node.py
ros2 run safety_node safety_node_cpp
```
If you want to see debug messages you can run:
```BASH
ros2 run safety_node safety_node --ros-args --log-level debug
```
With our **second terminal** open we can use our keyboard as prompted to control the car. Running into a wall should send warning messages to your **third terminal** and eventually stop your car in place.

## Conclusion
From the input data of the car's speed and surroundings we were able to implement an Emergency breaking system that works using debouncing to reduce false positives. Our `threshold_iTTC` and `debounce_threshold` can be changed to better optimize our breaking system in the future. Another potential problem with this breaking system is that it only works with head on collisions. If our car were to clip a corner while turning or to reverse into an object, our emergency breaking would not be able to detect the collision or stop it in time. This is due to the `ranges` coming from the position of the LiDAR sensor on the car. We would have to take into account the distance of the object to the closest point on the car instead of just the sensor in order to fix this issue. We can use the dimensions of the car and trig to account for this, however that is outside the scope of this lab.


