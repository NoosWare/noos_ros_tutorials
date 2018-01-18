# noos_ros_tutorials
ROS-based tutorials using the NOOS Cloud Robotics platform

## How to build it

```shell
git clone https://github.com/ortelio/noos_ros_tutorials.git
cd noos_ros_tutorials
catkin_make
source devel/setup.bash
```

If your Noos API is not installed in the default path `/usr/local`
use the following line:

```shell
catkin_make -DCMAKE_INSTALL_PREFIX=your_path
```

## Loop tutorial

Loop is a basic example about how to use Noos API in a simple loop
using the same object, but changing the content which is sent to the 
cloud platform.

The example sends an image every second to detect a face in the image.
An usb camera can be used to check the results, otherwise the same
image will be sent. The image can be found in `/data` folder as `lenna.png`.

To run it:

```shell
rosrun loop loop_node
```

## Vision Batch tutorial

Vision Batch example shows how to use the class `vision_batch` of the 
Noos API.

The example sends an image to the cloud platform every second to detect 
a face in the image. If a face is found, a second call is done to the platform,
but in this case, the image is cropped to send only the face for recogniting
the face expression and the age.

An usb camera can be used to check the results, otherwise the same
image will be sent. The image can be found in `/data` folder as `lenna.png`.

To run it:

```shell
rosrun vision_batch vision_batch_node
```

## SLAM tutorial

In this example shows how to do SLAM using a `laser` to create the map.

For doing SLAM, a configuration file needs to be loaded to the platform 
before sending the sensor data. It should be modified with your sensor
specification and the accuracy that you require. The default config file
is `/data/icp.ini`.

The program sends laser data to the cloud platform and it returns the 
pose of the robot in the map. The platform creates internally a map called
`example_map`. The name can be changed in the file `src/slam.cpp`.

To read the laser data other node is required which is going to publish 
the laser data in the topic `/scan`.
In this example is used the `RPlidar version 2` laser and their [ROS package](https://github.com/robopeak/rplidar_ros/tree/4a54ec7a333c3ebb3bc968c72d19d2ae49c8b28b).
It is a submodule, so for using it you need the next steps first:

```shell
git submodule init
git submodule update
catkin_make
```

### Run SLAM tutorial
If you have an `RPlidar v2` you can use the launch file that is created
in this example:

```shell
roslaunch slam slam_launch.launch
```

If you create your own laser node, run your node and then :

```shell
rosrun slam slam_node
```

## Dependecies

- ROS
- Noos API
- Boost >= 1.58 
- OpenCV >= 3.1
- OpenSSL
- Threads
