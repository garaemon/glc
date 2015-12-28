[glc](https://github.com/nullkey/glc( fork with [ROS](https://ros.org) binding.

You can record rviz by topic interface.

# Install

1. add ppa
```
sudo add-apt-repository ppa:arand/ppa
sudo apt-get update
```

2. Install libraries
```
sudo apt-get install libelfhacks-dev libpacketstream-dev libxxf86vm-dev
```

3. Checkout ros_glc to your workspace
```
wstool set --git ros_glc https://github.com/garaemon/ros-glc
wstool update ros_glc
```

4. Compile it
```
catkin b ros_glc
```

# Capture
Run rviz with glc-capture.
```
rosrun ros_glc glc-capture rosrun rviz rviz
```

Start recording by publishing `std_msgs/Empty` message to `/glc_start_capture`.

```
rostopic pub /glc_start_capture std_msgs/Empty
```

Stop recording by publishing `std_msgs/Empty` message to `/glc_stop_capture`.
```
rostopic pub /glc_stop_capture std_msgs/Empty
```

And you will get a glc file like `rviz-56770-0.glc`.

# Play back
Use glc-play command.
```
rosrun ros_glc glc-play rviz-56770-0.glc
```

# Encode
Use glc_encode.sh script.
```
rosrun ros_glc glc_encode.sh rviz-56770-0.glc
```
