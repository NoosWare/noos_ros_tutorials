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

###Run SLAM tutorial
If you have an `RPlidar v2` you can use the launch file that is created
in this example:

```shell
roslaunch slam slam_launch.launch
```

If you create your own laser node, run your node and then :

```shell
rosrun slam slam_node
```
