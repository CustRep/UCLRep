# UCLRep
Custom Repository of UCL.

Using Husky Robot for simulator.

## Dependency

Before begin our project, we recommended that you install following libraries:
```
sudo apt-get install ros-kinetic-octomap-ros ros-kinetic-rtabmap
```

## Usage

Under command just choose one to run, like #Simualtor or #RealBot
### Simualtor

Open terminal and typing under command:
```
roslaunch husky_gazebo husky_playpen.launch
```

Using under command for checkout topic:
```
rostopic echo /navsat/fix #check GPS message
rostopic echo /zed/rgb/image_raw #check camera image
rostopic echo /scan #check laser message
```
### RealBot

Open terminal and typing under command to bringup your own driver:
```
roslaunch <your_driver_package> <your_driver.launch>
roslaunch zed_wrapper zed.launch
#roslaunch gpslaunch 
#roslaunch laserlaunch
```

## Common Steps

### Navigation
```
roslaunch husky_rtabmap_config navigation.launch #simulator simulation:=true
roslaunch husky_rtabmap_config mapping_rviz.launch #simulator simulation:=true
```

### Keyboard Teleop
```
roslaunch husky_teleop keyboard_teleop.launch
```

### Checkout topic list
```
rostopic list
```

### Checkout node list
```
rosnode list
```

### Take photos
```
mkdir -p /home/$USER/Pictures/gazebo_photos
cd /home/$USER/Pictures/gazebo_photos
rosrun image_view image_view image:=/zed/rgb/image_raw
```
### Visual
```
rosrun rover_vision test_tennis_ball_test
```

### GPS
```
roslaunch gps_goal gps_goal.launch
roslaunch gps_goal initialize_origin.launch origin:=auto
rostopic pub /local_xy_origin geometry_msgs/PoseStamped ’{header: {frame_id: "/map " } , pose: {position: {x:-10 , y:-1 }}}’ -1
rostopic pub /gps_goal_fix sensor_msgs/NavSatFix "{latitude: 49.91, longitude: 8.90}" -1
```
OR
```
rostopic pub /gps_goal_pose geometry_msgs/PoseStamped '{ header: { frame_id : "/ map " } , pose: { position: { x: 43.658 , y: -79.379 } } } ' -1
```
