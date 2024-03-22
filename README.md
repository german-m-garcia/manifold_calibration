# manifold_calibration
ROS package for calibrating a differential drive mobile base with a Lidar sensor for the mk-ros project:
https://german-m-garcia.gitbook.io/mk-ros/

## Recording data

Your robot will normally have a static transformation being published for the Lidar frame of reference (e.g. "laser") with respect to the "base_link" frame. For the calibration routine to work this tf should not be published. Instead, a scan matcher should publish the transformation of the "laser" frame with respect to an odometry frame of reference, e.g. "odom". Additionally to the tf topic, you should record the encoder tics for each wheel. If you want to visualize the data you can also record the laser scans and other static tfs that you might have:
```
rosbag record tf tf_static scan /makeblock/lwheel /makeblock/rwheel 
```

## Launching the calibration node

A launch file is provided in the launch directory that uses the bag file provided in the data directory. The parameters to be set are the following:

| Parameter | Type | Desc |
| :---         |     :---:      |          ---: |
| left_encoder_topic   | string     | ROS topic for the left wheel encoder tics   |
| right_encoder_topic     | string       | ROS topic for the right wheel encoder tics      |
| iterations     | int       | Number of iterations for the solver   |
| radius_left     | double       | Left wheel's radius in m   |
| radius_right     | double       | Right wheel's radius in m     |
| tics_per_revolution     | int       | Encoder tics per wheel revolution     |
| baseline     | double       | Distance between wheel axes      |
| sensor_x     | double       | 2D Pose of the Lidar sensor. X offset in m     |
| sensor_y     | double       | 2D Pose of the Lidar sensor. Y offset in m      |
| sensor_theta     | double       | 2D Pose of the Lidar sensor. Angle in radians     |



# Publications
This is an implementation of the method described in <br>
Maurilio Di Cicco, Bartolomeo Della Corte and Giorgio Grisetti. "Unsupervised calibration of wheeled mobile platforms" 
In Proc. of the IEEE International Conference on Robotics and Automation (ICRA), Stockholm, 2016.
